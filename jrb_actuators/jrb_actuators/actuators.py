#!/usr/bin/env python3

from re import A
import traceback
from rclpy.node import Node
import rclpy
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
)
from std_msgs.msg import Bool, Float32
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


CAN_PROTOCOL_SERVO_ARM_LEFT_A = 0
CAN_PROTOCOL_SERVO_ARM_LEFT_B = 1
CAN_PROTOCOL_SERVO_ARM_LEFT_C = 2
CAN_PROTOCOL_SERVO_ARM_LEFT_D = 3
CAN_PROTOCOL_SERVO_ARM_LEFT_E = 4
CAN_PROTOCOL_SERVO_ARM_RIGHT_A = 5
CAN_PROTOCOL_SERVO_ARM_RIGHT_B = 6
CAN_PROTOCOL_SERVO_ARM_RIGHT_C = 7
CAN_PROTOCOL_SERVO_ARM_RIGHT_D = 8
CAN_PROTOCOL_SERVO_ARM_RIGHT_E = 9
CAN_PROTOCOL_SERVO_RAKE_LEFT_TOP = 10
CAN_PROTOCOL_SERVO_RAKE_LEFT_BOTTOM = 11
CAN_PROTOCOL_SERVO_RAKE_RIGHT_TOP = 12
CAN_PROTOCOL_SERVO_RAKE_RIGHT_BOTTOM = 13
CAN_PROTOCOL_SERVO_PUSH_ARM_LEFT = 14
CAN_PROTOCOL_SERVO_PUSH_ARM_RIGHT = 15
CAN_PROTOCOL_SERVO_MEASURE_FORK = 16
CAN_PROTOCOL_SERVO_PLIERS_INCLINATION = 17
CAN_PROTOCOL_SERVO_PLIERS = 18


def speed2rapportCyclique(speed):
    # speed entre -100 et 100
    # Si speed < 0 alors voltage A > voltage B
    return 0.025 * speed + 7


class Actuators_robotrouge(Node):
    def __init__(self):
        super().__init__("actuators")
        self.get_logger().info("init")

        # self.init_gpio()
        # self.init_serial()

        # Tf subscriber
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub_generic_command = self.create_publisher(
            ServoGenericCommand, "servo_generic_command", 10
        )

        self.pub_reboot_command = self.create_publisher(ServoID, "servo_reboot", 10)

        self.pub_actuator_state = self.create_publisher(
            JointState, "actuator_state", 10
        )

        self.pub_pump_left = self.create_publisher(PumpStatus, "left_pump_status", 10)

        self.pub_pump_right = self.create_publisher(PumpStatus, "right_pump_status", 10)

        self.pub_valve_left = self.create_publisher(
            ValveStatus, "left_valve_status", 10
        )

        self.pub_valve_right = self.create_publisher(
            ValveStatus, "right_valve_status", 10
        )

        self.sub_left_arm = self.create_subscription(
            PoseStamped, "left_arm_goto", partial(self.arm_goto_cb, "left"), 10
        )

        self.sub_right_arm = self.create_subscription(
            PoseStamped, "right_arm_goto", partial(self.arm_goto_cb, "right"), 10
        )

        self.sub_rakes = self.create_subscription(Bool, "open_rakes", self.rakes_cb, 10)

        self.sub_pump_left = self.create_subscription(
            Bool, "pump_left", partial(self.pump_cb, "left"), 10
        )
        self.sub_pump_right = self.create_subscription(
            Bool, "pump_right", partial(self.pump_cb, "right"), 10
        )

        self.sub_stack_sample = self.create_subscription(
            StackSample, "stack_sample", self.stackSample_cb, 10
        )

        self.init_actuators()

        publish_state_rate = 1 / 6  # Hz
        # self.state_publish_timer = self.create_timer(publish_state_rate, self.on_state_publish_timer)

        self.actuator_state_msg = JointState()

    def __del__(self):
        dxl.setTorque4All(self, 0)
        self.stopPump("left")
        self.stopPump("right")
        self.serial_actionBoard.close()

    def on_state_publish_timer(self):
        now = self.get_clock().now().to_msg()
        state = self.left_arm.getState()
        self.actuator_state_msg.header.stamp = now
        self.actuator_state_msg.name = [
            "left_arm_joint",
            "left_b_joint",
            "left_c_joint",
            "left_d_joint",
            "left_e_joint",
            "left_f_joint",
        ]
        self.actuator_state_msg.position = state

        self.pub_actuator_state.publish(self.actuator_state_msg)

    def sendGenericCommand(self, len_, id, addr, data):
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

        if(addr != 30):
            self.get_logger().info("addr not 30")
            self.get_logger().info(str(addr))


        if msg.id == 0:
            self.get_logger().error("msg.id == 0")
            return

        # while self.last_sending+0.0005>time.time():
        #    pass
        # self.last_sending=time.time()
        self.pub_generic_command.publish(msg)

    def sendRebootCommand(self, id):
        msg = ServoID()
        msg.id = id
        self.pub_reboot_command.publish(msg)

    def init_actuators(self):
        # self.last_sending=time.time()
        self.sendRebootCommand(254)  # 254 for broadcast
        time.sleep(10)

        self.left_arm = dxl.bras(self, "left", 16, 14, 22, 1, 8, 101)  # gauche
        self.right_arm = dxl.bras(self,"right",3, 4, 10, 9, 11, 100) #droit

        self.rateaux = dxl.rakes(self,7, 15, 5, 18)

        # centre reservoir : 112.75 ; -22.5
        x = 112.75
        y = -22.5

        self.startPump("left")
        time.sleep(2)
        # self.rateaux.setTorque(1)
        # self.rateaux.close()
        self.stopPump("left")
        self.left_arm.setTorque(1)
        # self.right_arm.setTorque(1)
        self.left_arm.setArmPosition(20, 120)
        # self.right_arm.setArmPosition(-20,120)
        time.sleep(1)
        # self.left_arm.initSlider()
        # self.right_arm.initSlider()
        self.left_arm.setTorque(1)
        # self.right_arm.setTorque(1)
        # self.rateaux.open()
        self.get_logger().info("init OK")

        # self.storeArm("left")
        # self.storeArm("right")
        time.sleep(2)
        # self.cycle_cool()

    def cycle_cool(self):
        while True:
            self.takeSample("left", 1)
            self.returnSample("left")

    def startPump(self, side):
        # self.serial_actionBoard.write(("pump "+side+" 1\r").encode('utf-8'))
        pump_msg = PumpStatus()
        pump_msg.enabled = True
        if side == "left":
            self.pub_pump_left.publish(pump_msg)
        else:
            self.pub_pump_right.publish(pump_msg)

    def stopPump(self, side):
        # self.serial_actionBoard.write(("valve "+side+" 1\r").encode('utf-8'))
        # self.serial_actionBoard.write(("pump "+side+" 0\r").encode('utf-8'))
        # time.sleep(0.1)
        # self.serial_actionBoard.write(("valve "+side+" 0\r").encode('utf-8'))
        pump_msg = PumpStatus()
        valve_msg = ValveStatus()
        pump_msg.enabled = False
        valve_msg.enabled = True
        if side == "left":
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
        z = 0.230 if z > 0.230 else z

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
            # self.right_arm.setSliderPosition_mm(z * 1000)
        elif side == "left":
            self.left_arm.setArmPosition(
                x * 1000,
                y * 1000,
                math.degrees(msg.pose.orientation.y),
                math.degrees(msg.pose.orientation.x),
                math.degrees(msg.pose.orientation.z),
            )
            # self.left_arm.setSliderPosition_mm(z * 1000)

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

    def lookupTransform(
        self, target_frame, source_frame, time=rclpy.time.Time().to_msg()
    ):
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


class Actuators_robotbleu(Node):
    def __init__(self):
        super().__init__("actuators")
        self.get_logger().info("init")

        self.xl320_status = {}
        self.present_resistance = 0

        self.sub_xl320_status = self.create_subscription(
            ServoStatus, "servo_status", self.xl320_status_cb, 10
        )
        self.sub_xl320_status = self.create_subscription(
            Float32, "resistance_status", self.present_resistance_cb, 10
        )
        self.pub_xl320_target = self.create_publisher(
            ServoAngle, "servo_angle_target", 10
        )
        self.pub_xl320_config = self.create_publisher(ServoConfig, "servo_config", 10)

        # config msg
        self.arm_left_config_msg = ServoConfig()
        self.arm_right_config_msg = ServoConfig()
        self.ohm_reader_config_msg = ServoConfig()
        self.plier_tilt_config_msg = ServoConfig()
        self.plier_config_msg = ServoConfig()

        self.init_actuators()

    def xl320_status_cb(self, msg: ServoStatus):
        if not msg.id in self.xl320_status:
            self.xl320_status[msg.id] = ServoStatus()
        self.xl320_status[msg.id] = msg

    def present_resistance_cb(self, msg: Float32):
        self.present_resistance = msg.data

    def init_actuators(self):
        # config servo
        self.arm_left_config_msg.id = CAN_PROTOCOL_SERVO_PUSH_ARM_LEFT
        self.arm_right_config_msg.id = CAN_PROTOCOL_SERVO_PUSH_ARM_RIGHT
        self.ohm_reader_config_msg.id = CAN_PROTOCOL_SERVO_MEASURE_FORK
        self.plier_tilt_config_msg.id = CAN_PROTOCOL_SERVO_PLIERS_INCLINATION
        self.plier_config_msg.id = CAN_PROTOCOL_SERVO_PLIERS

        self.arm_left_config_msg.torque_limit = (
            self.arm_right_config_msg.torque_limit
        ) = (
            self.ohm_reader_config_msg.torque_limit
        ) = self.plier_tilt_config_msg.torque_limit = 1023

        self.arm_left_config_msg.moving_speed = 512  # max: 1024
        self.arm_right_config_msg.moving_speed = 512  # max: 1024
        self.ohm_reader_config_msg.moving_speed = 300  # max: 1024
        self.plier_tilt_config_msg.moving_speed = 200  # max: 1024

        self.arm_left_config_msg.pid.pid = (
            self.arm_right_config_msg.pid.pid
        ) = self.ohm_reader_config_msg.pid.pid = self.plier_tilt_config_msg.pid.pid = [
            32.0,
            0.0,
            0.0,
        ]

        self.pub_xl320_config.publish(self.arm_left_config_msg)
        self.pub_xl320_config.publish(self.arm_right_config_msg)
        self.pub_xl320_config.publish(self.ohm_reader_config_msg)
        self.pub_xl320_config.publish(self.plier_tilt_config_msg)
        self.pub_xl320_config.publish(self.plier_config_msg)

        self.get_logger().info("configs published")
        time.sleep(3)

        # test servo
        self.setArm("left", "out")
        self.setArm("right", "out")
        self.setOhmReader(135)
        self.setPlierTilt("out")
        self.openPlier()
        time.sleep(1)
        self.setArm("left", "in")
        self.setArm("right", "in")
        self.setOhmReader(200)
        self.setPlierTilt("in")
        self.closePlier()
        time.sleep(1)

        # todo : servomoteur pince

        self.get_logger().info("init OK")

    def setArm(self, side, state):
        angle_msg = ServoAngle()

        if side == "left":
            angle_msg.id = self.arm_left_config_msg.id
            if state == "in":
                angle_msg.radian = math.radians(150)
            elif state == "out ":
                angle_msg.radian = math.radians(50)

        elif side == "right":
            angle_msg.id = self.arm_right_config_msg.id
            if state == "in":
                angle_msg.radian = math.radians(150)
            elif state == "out ":
                angle_msg.radian = math.radians(250)

        else:
            return

        self.pub_xl320_target.publish(angle_msg)

    def setOhmReader(self, degrees):
        angle_msg = ServoAngle()
        angle_msg.id = self.ohm_reader_config_msg.id
        angle_msg.radian = math.radians(degrees)
        self.pub_xl320_target.publish(angle_msg)

    def setPlierTilt(self, state):
        if state == "in":
            self.setPlierTiltAngle(245)
        elif state == "out":
            self.setPlierTiltAngle(155)

    def setPlierTiltAngle(self, degrees):
        angle_msg = ServoAngle()
        angle_msg.id = self.plier_tilt_config_msg.id
        angle_msg.radian = math.radians(degrees)
        self.pub_xl320_target.publish(angle_msg)

    def openPlier(self):
        angle_msg = ServoAngle()
        angle_msg.id = self.plier_config_msg.id
        angle_msg.radian = math.radians(15)
        self.pub_xl320_target.publish(angle_msg)

    def closePlier(self):
        angle_msg = ServoAngle()
        angle_msg.id = self.plier_config_msg.id
        angle_msg.radian = math.radians(50)
        self.pub_xl320_target.publish(angle_msg)


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