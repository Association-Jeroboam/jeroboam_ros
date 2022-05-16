import traceback
from rclpy.node import Node
import rclpy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from .lib import custom_dxl_API as API
from dynamixel_sdk import *  # Uses Dynamixel SDK library
import RPi.GPIO as GPIO
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

def speed2rapportCyclique(speed):
    # speed entre -100 et 100
    # Si speed < 0 alors voltage A > voltage B
    return 0.025 * speed + 7


channel_pompe = 12
channel_vanne = 35
frequence = 50


class Actuators(Node):
    def __init__(self):
        super().__init__("actuators")
        self.get_logger().info("init")

        self.init_gpio()
        self.init_actuators()

        # Tf subscriber
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sub_left_arm = self.create_subscription(
            PoseStamped, "left_arm_goto", partial(self.arm_goto_cb, "left"), 10
        )

        self.sub_right_arm = self.create_subscription(
            PoseStamped, "right_arm_goto", partial(self.arm_goto_cb, "right"), 10
        )

        self.sub_rakes = self.create_subscription(
            Bool, "open_rakes", self.rakes_cb, 10
        )

        self.pub_actuator_state = self.create_publisher(
            JointState, "actuator_state", 10
        )

        publish_state_rate = 1 / 6  # Hz
        self.state_publish_timer = self.create_timer(
            publish_state_rate, self.on_state_publish_timer
        )

        self.actuator_state_msg = JointState()

    def __del__(self):
        API.resetTorque4All()

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

    def init_gpio(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(channel_pompe, GPIO.OUT)
        GPIO.setup(channel_vanne, GPIO.OUT)

        self.pPompe = GPIO.PWM(channel_pompe, frequence)
        self.pVanne = GPIO.PWM(channel_vanne, frequence)

        self.pPompe.start(speed2rapportCyclique(0))
        self.pVanne.start(speed2rapportCyclique(0))

    def init_actuators(self):
        # Protocol version
        PROTOCOL_VERSION = 2.0  # See which protocol version is used in the Dynamixel

        # Default setting
        BAUDRATE = 57600  # Dynamixel default baudrate : 57600
        DEVICENAME = "/dev/ttyACM0"  # Check which port is being used on your controller
        # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

        API.initHandlers(DEVICENAME, BAUDRATE, PROTOCOL_VERSION)
        API.reboot(254)  # 254 for broadcast
        time.sleep(2)

        self.left_arm = API.bras("left",16, 14, 22, 1, 8, 101) #gauche
        self.right_arm = API.bras("right",3, 4, 10, 9, 11, 100) #droit

        self.rateaux = API.rakes(7, 15, 5, 18)

        # centre reservoir : 112.75 ; -22.5
        x = 112.75
        y = -22.5

        # self.startPump()
        self.rateaux.setTorque(1)
        self.rateaux.close()
        self.left_arm.setTorque(1)
        self.right_arm.setTorque(1)
        self.left_arm.setArmPosition(20,120)
        self.right_arm.setArmPosition(-20,120)
        time.sleep(1)
        self.left_arm.initSlider()
        self.right_arm.initSlider()
        self.left_arm.setTorque(1)
        self.right_arm.setTorque(1)
        self.rateaux.open()
        self.get_logger().info("init OK")

    def startPump(self):
        self.pPompe.ChangeDutyCycle(speed2rapportCyclique(100))
        self.pVanne.ChangeDutyCycle(speed2rapportCyclique(0))

    def stopPump(self):
        self.pPompe.ChangeDutyCycle(speed2rapportCyclique(0))
        self.pVanne.ChangeDutyCycle(speed2rapportCyclique(100))
        time.sleep(1)
        self.pVanne.ChangeDutyCycle(speed2rapportCyclique(0))

        # self.stopVanneTimer = self.create_timer(1, lambda: time.sleep(0.5))

        # def stopVanne():
        #     self.pVanne.ChangeDutyCycle(speed2rapportCyclique(0))
        #     self.stopVanneTimer.cancel()

    def rakes_cb(self, msg: Bool):
        if msg.data :
            self.rateaux.open()
        else :
            self.rateaux.close()

    def arm_goto_cb(self, side: str, msg: PoseStamped):
        #print("cb")
        pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, 1])

        if msg.header.frame_id != "left_arm_origin_link":
            transform = self.lookupTransform(
                "left_arm_origin_link", msg.header.frame_id, rclpy.time.Time().to_msg()
            )
            pos = transform.dot(pos)
            # print(pos)

        x = pos[0]
        y = pos[1]
        z = pos[2]
       
        z = 0 if z < 0 else z
        z = 0.230 if z > 0.230 else z

        self.get_logger().info(f"[GOTO] {side} ({x}, {y}, {z}) ({msg.pose.orientation.y}, {msg.pose.orientation.x}, {msg.pose.orientation.z})")

        # TODO : select arm
        self.left_arm.setArmPosition(x * 1000, y * 1000, math.degrees(msg.pose.orientation.y),math.degrees(msg.pose.orientation.x),math.degrees(msg.pose.orientation.z))
        self.left_arm.setSliderPosition_mm(z * 1000)


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


def main(args=None):
    rclpy.init(args=args)
    node = Actuators()

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
