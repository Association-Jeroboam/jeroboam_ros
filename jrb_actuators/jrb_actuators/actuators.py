#!/usr/bin/env python3

USB_BOARD = True
MIN_TIME_BETWEEN_COM = 0.007 #seconds

from re import A
import traceback
from rclpy.node import Node
import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from geometry_msgs.msg import PoseStamped
from jrb_msgs.msg import (
    StackSample,
    PumpStatus,
    ValveStatus,
    ServoStatus,
    ServoConfig,
    ServoAngle,
    ServoGenericCommand,
    ServoID,
    BoolArray,
    ArmStatus,
    SimplifiedGoalStatus,

)
from std_msgs.msg import Bool, Float32, Int8, Int16, UInt16, String
from jrb_actuators.lib import dxl
import time
import numpy as np
from functools import partial
from sensor_msgs.msg import JointState
from tf_transformations import (
    concatenate_matrices,
    translation_matrix,
    quaternion_matrix,
)
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import math

if USB_BOARD : from dynamixel_sdk import *

class Actuators(Node):
    def __init__(self):
        super().__init__("actuators")
        self.get_logger().info("init")

        self.actuatorsInitialized = False

        # Tf subscriber
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.last_com_time = 0

        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_ALL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE 
        )

        if USB_BOARD :
            self.get_logger().info("Using USB board for dynamixel control")
            self.servo_angle_publish_timer = self.create_timer( 0.5, self.on_servo_angle_publish_timer)
            self.pub_servo_angle = self.create_publisher(ServoAngle, "servo_angle", 10 )
            self.initHandlers("/dev/dxl", 57600, 2.0)
        else :
            self.get_logger().info("Using can bus for dynamixel control")    
            self.pub_generic_command = self.create_publisher(ServoGenericCommand, "servo_generic_command", qos_profile)
            self.pub_reboot_command = self.create_publisher(ServoID, "servo_reboot", qos_profile)
            self.sub_servo_angle = self.create_subscription(ServoAngle, "servo_angle", self.servoAngle_cb, 10) #pas besoin en usb car actuators connais déjà la position

        self.sub_action_launch = self.create_subscription(String, "actuators/generic_action_launch", self.actionLaunch_cb, qos_profile)

        self.pub_action_status = self.create_publisher(SimplifiedGoalStatus, "actuators/generic_action_status", qos_profile)
        self.action_status_publish_timer = self.create_timer(0.2, self.on_action_status_publish_timer)

        self.actions_running = {}

    def __del__(self):
        dxl.setTorque4All(self, 0)

    def on_servo_angle_publish_timer(self):
        if not self.actuatorsInitialized :
            return
        msg = ServoAngle()
        servo_to_remove = []

        for servo_id in dxl.connected_XL320 :
            if dxl.connected_XL320[servo_id].isReady :
                value=dxl.connected_XL320[servo_id].getPresentPosition()
                if value != -1 :
                    msg.id = servo_id
                    msg.radian = value*dxl.rawToRad_XL320
                    self.pub_servo_angle.publish(msg)
                else :
                    self.get_logger().error(f"Unable to communicate with DXL {servo_id}")
                    servo_to_remove.append(servo_id)

        for servo_id in dxl.connected_XL430 :
            if dxl.connected_XL430[servo_id].isReady :
                value=dxl.connected_XL430[servo_id].getPresentPosition()
                if value != -1 :
                    msg.id = servo_id
                    msg.radian = float(value) #*dxl.rawToRad_XL430
                    self.pub_servo_angle.publish(msg)
                else :
                    self.get_logger().error(f"Unable to communicate with DXL {servo_id}")
                    servo_to_remove.append(servo_id)

        for servo  in servo_to_remove :
            #del dxl.connected_XL320[servo]
            pass

    def on_action_status_publish_timer(self):
        msg =  SimplifiedGoalStatus()
        actions_to_close = []
        for action_name, servos in self.actions_running.items() :
            msg.action_name = action_name
            msg.status = SimplifiedGoalStatus.STATUS_SUCCEEDED
            for servo in servos :
                if not servo.torque :
                    msg.status = SimplifiedGoalStatus.STATUS_ABORTED
                    actions_to_close.append(action_name)
                    break
                elif not servo.target_reached :
                    msg.status = SimplifiedGoalStatus.STATUS_EXECUTING
                    #self.get_logger().info(f"XL320 with ID {servo.ID} has gap of {abs(servo.radian_target-servo.radian_state)}.")
                    break
            if msg.status == SimplifiedGoalStatus.STATUS_SUCCEEDED :
                actions_to_close.append(action_name)
                #self.get_logger().info(f"Action {action_name} succeed !")
            self.pub_action_status.publish(msg)
        for name in actions_to_close :
            del self.actions_running[name]

    def initHandlers(self, DEVICENAME, BAUDRATE, PROTOCOL_VERSION):
        # Initialize PortHandler instance
        # Set the port pathsudo ap
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        self.portHandler = PortHandler(DEVICENAME)

        # Initialize PacketHandler instance
        # Set the protocol version
        # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        # Open port
        if self.portHandler.openPort():
            #self.get_logger().info("Succeeded to open the port")
            pass
        else:
            self.get_logger().error("Failed to open the port")
            return

        # Set port baudrate
        if self.portHandler.setBaudRate(BAUDRATE):
            #self.get_logger().info("Succeeded to change the baudrate")
            pass
        else:
            self.get_logger().error("Failed to change the baudrate")
            return

    def pingDXL(self,DXL_ID):
        PROTOCOL_VERSION=2
        dxl_model_number, dxl_comm_result, dxl_error = self.packetHandler.ping(self.portHandler,DXL_ID)
        if dxl_comm_result != COMM_SUCCESS:
            #self.get_logger().error("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            pass
        elif dxl_error != 0:
            #self.get_logger().error("%s" % self.packetHandler.getRxPacketError(dxl_error))
            pass
        else:
            #self.get_logger().info("[ID:%03d] ping Succeeded. Dynamixel model number : %d" % (DXL_ID, dxl_model_number))
            pass
        return dxl_model_number

    def readValue(self, size, ID, address):
        if ( ID in dxl.connected_XL320 ) or ( ID in dxl.connected_XL430 ):
            while time.time() - self.last_com_time < MIN_TIME_BETWEEN_COM :
                pass
            self.last_com_time=time.time()

            if size == 1:
                value, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(
                    self.portHandler, ID, address
                )
                if dxl_comm_result != COMM_SUCCESS:
                    
                    value, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(
                        self.portHandler, ID, address
                    )
            elif size == 2:
                value, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(
                    self.portHandler, ID, address
                )
                if dxl_comm_result != COMM_SUCCESS:
                    
                    value, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(
                        self.portHandler, ID, address
                    )
            elif size == 3:
                value, dxl_comm_result, dxl_error = self.packetHandler.read3ByteTxRx(
                    self.portHandler, ID, address
                )
                if dxl_comm_result != COMM_SUCCESS:
                    
                    value, dxl_comm_result, dxl_error = self.packetHandler.read3ByteTxRx(
                        self.portHandler, ID, address
                    )
            elif size == 4:
                value, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(
                    self.portHandler, ID, address
                )
                if dxl_comm_result != COMM_SUCCESS:
                    
                    value, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(
                        self.portHandler, ID, address
                    )
            else:
                self.get_logger().error("Incorrect size")
                return -1

            if dxl_comm_result != COMM_SUCCESS:
                if address != 37:
                    self.get_logger().warn(f"Comm_result for ID {ID} address {address} : {self.packetHandler.getTxRxResult(dxl_comm_result)}")
                pass
            elif dxl_error != 0:
                if ID in dxl.connected_XL320 : #C'est pas un XL430
                    if dxl_error == 128 : #hardware error
                        if address == 50:
                            if (value >> 0) & 1 : self.get_logger().error(f"DXL error for ID {ID} : Overload")
                            if (value >> 1) & 1 : self.get_logger().error(f"DXL error for ID {ID} : Overheating")
                            if (value >> 2) & 1 : self.get_logger().error(f"DXL error for ID {ID} : Input voltage error")
                        else :
                            self.readValue(1,ID,50)
                    else :
                        self.get_logger().warn(f"DXL error for ID {ID} address {address} : {self.packetHandler.getRxPacketError(dxl_error)} (error:{dxl_error} / value:{value})")
            else:
                value = dxl.toSigned(value,size*8)
                return value
        elif ID==254 :
            self.get_logger().warn(f"Unable to readValue on all DXL at the same time (ID=254)")
        else :
            self.get_logger().warn(f"ID {ID} is not in connected_XL320 or connected_XL430")
        return -1

    def writeValue(self, size, ID, address, value):
        if ( ID in dxl.connected_XL320 ) or ( ID in dxl.connected_XL430 ) or ID==254:
            value=dxl.toUnsigned(value,size*8)
            #self.get_logger().info(f"Writing on ID {ID}  address {address}  value {value}  size {size}")
            if size == 1:
                dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
                    self.portHandler, ID, address, value
                )
            elif size == 2:
                dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(
                    self.portHandler, ID, address, value
                )
            elif size == 3:
                dxl_comm_result, dxl_error = self.packetHandler.write3ByteTxRx(
                    self.portHandler, ID, address, value
                )
            elif size == 4:
                dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
                    self.portHandler, ID, address, value
                )
            else:
                self.get_logger().error("writeValue : Incorrect size")
                return

            if dxl_comm_result != COMM_SUCCESS:
                # print(
                #     "Comm_result for ID",
                #     ID,
                #     " address",
                #     address,
                #     ": %s" % packetHandler.getTxRxResult(dxl_comm_result),
                # )
                pass
            elif dxl_error != 0:
                error_msg=self.packetHandler.getRxPacketError(dxl_error)
                if(error_msg == "[RxPacketError] Hardware error occurred. Check the error at Control Table (Hardware Error Status)!"):
                    if ID in dxl.connected_XL320 : #C'est pas un XL430
                        self.readValue(1,ID,50)
                        return

                self.get_logger().error(f"DXL error for ID {ID} address {address} (value {value}) : {error_msg}")
                if(error_msg == "[RxPacketError] The data value exceeds the limit value!"):
                    self.get_logger().error(f"value :{value}  size :{size}")

            #readValue = self.readValue(size,ID,address)
            #if readValue != value :
            #    self.get_logger().error(f"Write error for DXL ID {ID} address {address} (value sent {value} / value read {readValue})")

    def sendGenericCommand(self, len_, id, addr, data):
        while time.time() - self.last_com_time < MIN_TIME_BETWEEN_COM :
                pass
        self.last_com_time=time.time()

        if USB_BOARD :
            self.writeValue(len_, id, addr, data)
        else :
            data_array = []
            while data > 0:
                data_array.append(data % 255)
                data = data // 255

            if len(data_array) > len_:
                self.get_logger().warn("Data does not fit on specified command lenght")

            while len(data_array) < 8:
                data_array.append(0)

            msg = ServoGenericCommand()
            msg.id = id
            msg.addr = addr
            msg.len = len_
            msg.data = data_array

            #if(addr != 30):
            #    self.get_logger().info("addr not 30")
            #    self.get_logger().info(str(addr))


            if msg.id == 0:
                self.get_logger().error("msg.id == 0")
                return

            # while self.last_sending+0.0005>time.time():
            #    pass
            # self.last_sending=time.time()
            self.get_logger().info(f"Publishing GenericCommand id={id}")
            self.pub_generic_command.publish(msg)

    def sendRebootCommand(self, id):
        if USB_BOARD :
                self.packetHandler.reboot(self.portHandler, id)
                time.sleep(2)
        else :
            msg = ServoID()
            msg.id = id
            self.pub_reboot_command.publish(msg)

    def servoAngle_cb(self, msg: ServoAngle):
        if msg.id in dxl.connected_XL320:
            dxl.connected_XL320[msg.id].saveCurrentPosition(msg.radian,isRadian=True)
            #self.get_logger().info(f"XL320 with ID {msg.id} is at {msg.radian} radians.")
        else:
            self.get_logger().debug(f"XL320 with ID {msg.id} is unknown.")

    def actionLaunch_cb(self, msg : String):
        if msg.data[0] == "/" : # Abort the action requested by the server
            action_name=msg.data.lstrip('/')
            if action_name in self.actions_running :
                del self.actions_running[action_name]
        else :
            self.get_logger().warn("Cannot launch unknown action")

    def init_actuators(self):
        self.actuatorsInitialized

class Actuators_robotrouge(Actuators):
    def __init__(self):
        super().__init__()

        self.pub_arm_state_left = self.create_publisher(
            ArmStatus, "arm_state_left", 10
        )

        self.pub_arm_state_right = self.create_publisher(
            ArmStatus, "arm_state_right", 10
        )

        self.pub_pump_left = self.create_publisher(
            Bool, "/hardware/pump/left/set_status", 10)
        )

        self.pub_pump_right = self.create_publisher(
            PumpStatus, "/hardware/pump/right/set_status", 10)

        self.pub_valve_left = self.create_publisher(
            Bool, "/hardware/valve/left/set_status", 10
        )

        self.pub_valve_right = self.create_publisher(
            Bool, "/hardware/valve/right/set_status", 10
        )

        # Subscribers
        self.sub_left_arm = self.create_subscription(
            PoseStamped, "/actuators/arm/left/go_to", partial(self.arm_goto_cb, "left"), 10
        )

        self.sub_right_arm = self.create_subscription(
            PoseStamped, "/actuators/arm/right/go_to", partial(self.arm_goto_cb, "right"), 10
        )

        self.sub_rakes = self.create_subscription(
            Bool, "/actuators/set_open_rakes", self.rakes_cb, 10
        )

        self.sub_pump_left = self.create_subscription(
            Bool, "/actuators/pump/left/set_enabled", partial(self.pump_cb, "left"), 10
        )

        self.sub_pump_right = self.create_subscription(
            Bool, "/actuators/pump/right/set_enabled", partial(self.pump_cb, "right"), 10
        )

        # TODO: @lucas see if this is still relevant
        # self.sub_stack_sample = self.create_subscription(
        #     StackSample, "stack_sample", self.stackSample_cb, 10
        # )

        arm_publish_state_rate = 1 / 2  # Hz
        self.arm_state_publish_timer = self.create_timer(arm_publish_state_rate, self.on_arm_state_publish_timer)

        self.arm_state_msg = ArmStatus()

        self.init_actuators()

        self.get_logger().info("init OK")

    def __del__(self):
        dxl.setTorque4All(self, 0)
        self.stopPump("left")
        self.stopPump("right")

    def on_arm_state_publish_timer(self):
        now = self.get_clock().now().to_msg()
        self.arm_state_msg.header.stamp = now
        self.arm_state_msg.name = [
            "slider",
            "a_joint",
            "b_joint",
            "c_joint",
            "d_joint",
            "e_joint",
        ]

        #left
        self.arm_state_msg.gap = self.left_arm.getGapAngles()
        self.arm_state_msg.target_reached = self.left_arm.getJoinStatus() 
        self.pub_arm_state_left.publish(self.arm_state_msg)
        
        #right
        self.arm_state_msg.gap = self.right_arm.getGapAngles()
        self.arm_state_msg.target_reached = self.right_arm.getJoinStatus() 
        self.pub_arm_state_right.publish(self.arm_state_msg)

    def init_actuators(self):
        # self.last_sending=time.time()
        self.sendRebootCommand(254)  # 254 for broadcast

        self.rateaux = dxl.rakes(self,7, 15, 5, 18)

        self.left_arm = dxl.bras(self, "left", 16, 14, 22, 1, 8, 101)  # gauche
        self.right_arm = dxl.bras(self,"right",3, 4, 10, 9, 11, 100) #droit

        # centre reservoir : 112.75 ; -22.5
        x = 112.75
        y = -22.5

        self.startPump("left")
        #time.sleep(2)
        # self.rateaux.setTorque(1)
        # self.rateaux.close()
        self.stopPump("left")
        self.left_arm.setTorque(1)
        # self.right_arm.setTorque(1)
        #self.left_arm.setArmPosition(20, 120)
        # self.right_arm.setArmPosition(-20,120)
        #time.sleep(1)
        self.left_arm.initSlider()
        self.right_arm.initSlider()
        #self.left_arm.setTorque(1)
        # self.right_arm.setTorque(1)
        # self.rateaux.open()

        # self.storeArm("left")
        # self.storeArm("right")
        #time.sleep(2)
        # self.cycle_cool()
        self.left_arm.slider.setTorque(True)
        self.right_arm.slider.setTorque(True)
        self.actuatorsInitialized = True
        self.right_arm.setSliderPosition_mm(100)

    def cycle_cool(self):
        while True:
            self.takeSample("left", 1)
            self.returnSample("left")

    def startPump(self, side):
        #self.serial_actionBoard.write(("pump "+side+" 1\r").encode('utf-8'))
        pump_msg=Bool(data=True)
        if side == "left" :
            self.pub_pump_left.publish(pump_msg)
        else:
            self.pub_pump_right.publish(pump_msg)

    def stopPump(self, side):
        #self.serial_actionBoard.write(("valve "+side+" 1\r").encode('utf-8'))
        #self.serial_actionBoard.write(("pump "+side+" 0\r").encode('utf-8'))
        #time.sleep(0.1)
        #self.serial_actionBoard.write(("valve "+side+" 0\r").encode('utf-8'))
        pump_msg=Bool(data=True)
        valve_msg=Bool(data=True)
        if side == "left" :
            self.pub_pump_left.publish(pump_msg)
            self.pub_valve_left.publish(valve_msg)
        else:
            self.pub_pump_right.publish(pump_msg)
            self.pub_valve_right.publish(valve_msg)

    def pump_cb(self, side: str, msg: Bool):
        if msg.data:
            self.startPump(side)
        else:
            self.stopPump(side)

    def rakes_cb(self, msg: Bool):
        if msg.data:
            self.rateaux.open()
        else:
            self.rateaux.close()

    def arm_goto_cb(self, side: str, msg: PoseStamped):
        pos = np.array(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, 1]
        )

        if msg.header.frame_id != side + "_arm_origin_link":
            transform = self.lookupTransform(
                side + "_arm_origin_link",
                msg.header.frame_id,
                rclpy.time.Time().to_msg(),
            )
            pos = transform.dot(pos)

        x = pos[0]
        y = pos[1]
        z = pos[2]

        z = 0 if z < 0 else z
        z = 0.215 if z > 0.215 else z

        self.get_logger().info(
            f"[GOTO] {side} ({x}, {y}, {z}) ({math.degrees(msg.pose.orientation.y)}, {math.degrees(msg.pose.orientation.x)}, {math.degrees(msg.pose.orientation.z)})"
        )

        if side == "right":
            self.right_arm.setArmPosition(
                x * 1000,
                y * 1000,
                math.degrees(msg.pose.orientation.y),
                math.degrees(msg.pose.orientation.x),
                math.degrees(msg.pose.orientation.z),
            )
            self.right_arm.setSliderPosition_mm(z * 1000)
        elif side == "left":
            self.left_arm.setArmPosition(
                x * 1000,
                y * 1000,
                math.degrees(msg.pose.orientation.y),
                math.degrees(msg.pose.orientation.x),
                math.degrees(msg.pose.orientation.z),
            )
            self.left_arm.setSliderPosition_mm(z * 1000)

    def stackSample_cb(self, msg: StackSample):
        self.stackSample(msg.side, msg.sample_index)
        # self.returnSample(msg.side)

    def returnSample(self, side):
        self.rateaux.close()
        self.left_arm.setSliderPosition_mm(200)
        self.right_arm.setSliderPosition_mm(200)

        if side == "left":
            self.left_arm.setArmPosition(0, 130, 0, 90, 0)
            self.right_arm.setArmPosition(0, 130, 0, 90, 0)
            self.left_arm.joinD.setGoalPosition(310)
            self.left_arm.joinE.setGoalPosition(512)
            self.right_arm.joinD.setGoalPosition(280)
            self.right_arm.joinE.setGoalPosition(512)
            time.sleep(0.8)

            self.left_arm.joinA.setGoalPosition(1024 - 323)
            self.left_arm.joinB.setGoalPosition(1024 - 530)
            self.left_arm.joinC.setGoalPosition(500)
            time.sleep(0.5)
            self.right_arm.joinA.setGoalPosition(335)
            self.right_arm.joinB.setGoalPosition(530)
            self.right_arm.joinC.setGoalPosition(490)

            time.sleep(1)
            self.startPump("right")
            time.sleep(1)
            self.stopPump(side)
            time.sleep(1)

            self.storeArm(side)
            time.sleep(1)
            self.right_arm.setArmPosition(0, 130, 0, 90, 0)
            time.sleep(1)
            self.stackSample("right", 1)

            print("stack ok")

    def takeSample(self, side, sample_index):
        z = 55 + (15 * (sample_index + 1))

        self.left_arm.setArmPosition(0, 130, 0, 90, 0)
        self.left_arm.setSliderPosition_mm(z + 20)
        time.sleep(1)
        self.rateaux.open()
        time.sleep(1)
        self.left_arm.setArmPosition(112.75, -22.5, 0, 90, -65)
        time.sleep(1)
        self.left_arm.setSliderPosition_mm(z)
        self.startPump("left")
        time.sleep(1)
        self.left_arm.setSliderPosition_mm(z + 20)
        time.sleep(0.5)
        self.left_arm.setArmPosition(0, 130, 0, 90, 0)
        time.sleep(1)

    def stackSample(self, side, sample_index):
        # todo : hauteur du slider selon nombre de sample dans le reservoir
        # todo : aligner E pour orienter l'echantillon comme il faut selon la camera => on va le faire au moment de la saisie, pas au moment de la dépose

        z = 55 + 10 + (15 * (sample_index + 1))

        if side == "left":
            self.left_arm.setSliderPosition_mm(z)
            self.left_arm.slider.waitMoveEnd(10)
            self.rateaux.open()
            self.left_arm.setArmPosition(112.75, -22.5, 0, 90, -65)
            time.sleep(1)
            self.stopPump(side)
            # todo pareil que pour right

        else:
            self.right_arm.setSliderPosition_mm(z + 10)
            self.right_arm.slider.waitMoveEnd(10)
            self.rateaux.open()
            self.right_arm.setArmPosition(-112.75, -22.5, 0, 90, -65)
            time.sleep(2)
            self.stopPump(side)
            self.right_arm.setSliderPosition_mm(z + 20)
            time.sleep(1)
            self.right_arm.setArmPosition(0, 130, 0, 90, -65)
            time.sleep(2)

            # bourrage
            self.right_arm.setSliderPosition_mm(z)
            self.right_arm.setArmPosition(20, 110, 40, 90, 0)
            time.sleep(2)
            self.right_arm.setArmPosition(-97, 103, 40, 90, 0)

            # right (-0.10400000000000008, 0.10099999999999998, 0.09999999999999992) (0.4363323129985824, 1.5707963267948966, 0.0)
            # right (-0.09700000000000007, 0.10299999999999998, 0.10499999999999993) (0.5235987755982988, 1.5707963267948966, 0.0)

        # time.sleep(2)
        # self.storeArm(side)
        self.rateaux.close()

    def storeArm(self, side):
        if side == "left":
            self.left_arm.setArmPosition(-88, 47, 0, -90, 0)
            time.sleep(0.8)
            self.left_arm.setArmPosition(-88, 47, 90, -90, 0)
        else:
            self.right_arm.setArmPosition(88, 47, 0, -90, 0)
            time.sleep(0.8)
            self.right_arm.setArmPosition(88, 47, 90, -90, 0)

    def lookupTransform(self, target_frame, source_frame, time=rclpy.time.Time().to_msg()):
        transform_msg = self.tf_buffer.lookup_transform(
            target_frame, source_frame, time
        )
        trans = np.array(
            [
                transform_msg.transform.translation.x,
                transform_msg.transform.translation.y,
                transform_msg.transform.translation.z,
            ]
        )
        rot = np.array(
            [
                transform_msg.transform.rotation.x,
                transform_msg.transform.rotation.y,
                transform_msg.transform.rotation.z,
                transform_msg.transform.rotation.w,
            ]
        )
        transform = concatenate_matrices(
            translation_matrix(trans), quaternion_matrix(rot)
        )

        return transform

class Actuators_robotbleu(Actuators):
    def __init__(self):
        super().__init__()

        self.sub_roll_speed = self.create_subscription(Int8, "hardware/roll/speed", self.roll_speed_cb, 10)
        self.sub_roll_height = self.create_subscription(Int16, "hardware/roll/height", self.roll_height_cb, 10)
        self.sub_turbine_speed = self.create_subscription(UInt16, "hardware/turbine/speed", self.turbine_speed_cb, 10)
        self.get_logger().info("init OK")

    def init_actuators(self):
        self.sendRebootCommand(254)  # 254 for broadcast
        time.sleep(1)
        self.ballSystem = dxl.ball_system(self)
        self.ballSystem.setTorque(True)
        self.actuatorsInitialized = True

    def actionLaunch_cb(self, msg : String):
        if msg.data[0] == "/" : # Abort the action requested by the server
            action_name=msg.data.lstrip('/')
            if action_name in self.actions_running :
                del self.actions_running[action_name]

        # Roller height
        elif msg.data == "SET_ROLLER_UP" :
            self.actions_running[msg.data] = self.ballSystem.setRollerUp()
        elif msg.data == "SET_ROLLER_DOWN" :
            self.actions_running[msg.data] = self.ballSystem.setRollerDown()
        elif msg.data == "SET_ROLLER_MIDDLE" :
            self.actions_running[msg.data] = self.ballSystem.setRollerMiddle()

        # Roller speed
        elif msg.data == "START_ROLLER_IN" :
            self.actions_running[msg.data] = self.ballSystem.startRoller_in()
        elif msg.data == "START_ROLLER_OUT" :
            self.actions_running[msg.data] = self.ballSystem.startRoller_out()
        elif msg.data == "STOP_ROLLER" :
            self.actions_running[msg.data] = self.ballSystem.stopRoller()
        
        # Other
        else :
            self.get_logger().warn(f"Cannot launch unknown action ({msg.data})")    

    def roll_speed_cb(self, msg: Int8):
        if msg.data == 1 :
            self.ballSystem.startRoller_in()
        elif msg.data == -1 :
            self.ballSystem.startRoller_out()
        else :
            self.ballSystem.stopRoller()

    def roll_height_cb(self, msg: Int16):
        if msg.data == 0 :
            self.ballSystem.setRollerDown()
        elif msg.data == 1 :
            self.ballSystem.setRollerMiddle()
        else :
            self.ballSystem.setRollerUp()

    def turbine_speed_cb(self, msg: UInt16):
        if msg.data > 0 :
            self.ballSystem.startFingersLoop()
        else :
            self.ballSystem.stopFingersLoop()
            self.ballSystem.setFingersOnCenter()

def main(args=None):
    rclpy.init(args=args)
    # node = Actuators()
    
    node = Actuators_robotrouge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped cleanly")
    except Exception:
        print("Error while stopping the node")
        print(traceback.format_exc())
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()