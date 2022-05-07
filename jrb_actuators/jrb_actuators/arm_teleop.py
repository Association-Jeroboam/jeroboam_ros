#!/usr/bin/env python
# coding: utf-8

import os
import time

if os.name == "nt":
    import msvcrt

    def getch():
        return msvcrt.getch().decode()

else:
    import sys, tty, termios

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


import traceback
from rclpy.node import Node
import rclpy
from geometry_msgs.msg import Pose
import time

class ArmTeleop(Node):
    def __init__(self):
        super().__init__("arm_teleop")
        self.get_logger().info("init")

        self.pub_goto = self.create_publisher(
            Pose, "left_arm_goto", 10
        )

        # centre reservoir : 112.75 ; -22.5
        self.goto_msg = Pose()
        self.goto_msg.position.x = 0.1127
        self.goto_msg.position.y = -0.0225
        self.goto_msg.position.z = 0.1

    def loop(self):
        self.get_logger().info("Press any key to continue! (or press ESC to quit!)")
        while rclpy.ok():
            value = getch()
            if value == chr(0x1B):
                break
            elif value == "q":
                self.goto_msg.position.x += 0.01
            elif value == "d":
                self.goto_msg.position.x -= 0.01
            elif value == "z":
                self.goto_msg.position.y -= 0.01
            elif value == "s":
                self.goto_msg.position.y += 0.01

            self.get_logger().info(f"goto {self.goto_msg.position.x} {self.goto_msg.position.y} {self.goto_msg.position.z}")
            self.pub_goto.publish(self.goto_msg)

        self.get_logger().info("Node stopped cleanly")

def main(args=None):
    rclpy.init(args=args)
    node = ArmTeleop()

    try:
        node.loop()
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

