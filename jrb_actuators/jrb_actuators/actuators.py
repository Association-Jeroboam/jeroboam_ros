import traceback
from rclpy.node import Node
import rclpy
from geometry_msgs.msg import Pose
from .lib import custom_dxl_API as API
from dynamixel_sdk import *  # Uses Dynamixel SDK library
import RPi.GPIO as GPIO
import time
from functools import partial
from sensor_msgs.msg import JointState


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

        self.sub_left_arm = self.create_subscription(
            Pose, "left_arm_goto", partial(self.arm_goto_cb, "left"), 10
        )

        self.sub_right_arm = self.create_subscription(
            Pose, "right_arm_goto", partial(self.arm_goto_cb, "right"), 10
        )

        self.pub_actuator_state = self.create_publisher(
            JointState, "actuator_state", 10
        )

        publish_state_rate = 1 / 10  # Hz
        self.state_publish_timer = self.create_timer(
            publish_state_rate, self.on_state_publish_timer
        )

        self.actuator_state_msg = JointState()

    def on_state_publish_timer(self):
        now = self.get_clock().now().to_msg()
        state = self.bras.getState()
        state[0] = state[0] / 1000.0 # mm -> m

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

        self.bras = API.bras(16, 14, 22, 1, 8, 2)

        self.rateaux = API.rakes()

        # centre reservoir : 112.75 ; -22.5
        x = 112.75
        y = -22.5

        # self.startPump()

        # self.rateaux.setTorque(1)
        # self.rateaux.close()
        self.bras.initSlider()
        self.bras.slider.setTorque(0)
        self.bras.setTorque(1)

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

    def arm_goto_cb(self, side: str, msg: Pose):
        x = msg.position.x
        y = msg.position.y
        z = msg.position.z

        self.get_logger().info(f"[GOTO] {side} ({x}, {y}, {z})")

        # TODO : select arm
        self.bras.setArmPosition(x*1000, y*1000)


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
