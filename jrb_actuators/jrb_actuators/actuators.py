import traceback
from rclpy.node import Node
import rclpy
from geometry_msgs.msg import Pose
from lib import custom_dxl_API as API
from dynamixel_sdk import *  # Uses Dynamixel SDK library
import RPi.GPIO as GPIO
import time
from functools import partial


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

        self.sub_left_arm = self.create_subscription(
            Pose, "left_arm_goto", partial(self.arm_goto_cb, "left"), 10
        )

        self.sub_right_arm = self.create_subscription(
            Pose, "right_arm_goto", partial(self.arm_goto_cb, "right"), 10
        )

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
        # bras.setTorque(1)

        # centre reservoir : 112.75 ; -22.5
        x = 112.75
        y = -22.5

        self.startPump()

        self.rateaux.setTorque(1)
        self.rateaux.close()
        self.bras.initSlider()

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
        x = msg.x
        y = msg.y
        self.get_logger().info(f"[GOTO] {side} ({x}, {y})")

        # TODO : select arm
        self.arm.setArmPosition(x, y)


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
        node.cleanup()


if __name__ == "__main__":
    main()
