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
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

import time


class ArmTeleop(Node):
    def __init__(self):
        super().__init__("arm_teleop")
        self.get_logger().info("init")

        self.pub_goto = self.create_publisher(PoseStamped, "left_arm_goto", 10)
        self.pub_rakes = self.create_publisher(Bool, "open_rakes",10)

        self.goto_msg = PoseStamped()
        self.goto_msg.header.frame_id = "left_arm_origin_link"
        self.goto_msg.pose.position.x = 0.0
        self.goto_msg.pose.position.y = 0.13
        self.goto_msg.pose.position.z = 0.22

        self.open_rakes_msg = Bool()

    def loop(self):
        self.get_logger().info("Press any key to continue! (or press ESC to quit!)")
        while rclpy.ok():
            publisherID=1
            value = getch()
            if value == chr(0x1B):
                break
            elif value == "q":
                self.goto_msg.pose.position.x -= 0.001
            elif value == "d":
                self.goto_msg.pose.position.x += 0.001
            elif value == "z":
                self.goto_msg.pose.position.y += 0.001
            elif value == "s":
                self.goto_msg.pose.position.y -= 0.001
            elif value == "a":
                self.goto_msg.pose.position.z -= 0.005
            elif value == "e":
                self.goto_msg.pose.position.z += 0.005
            elif value == "h": #homing
                self.goto_msg.pose.position.x = 0.0
                self.goto_msg.pose.position.y = 0.13

            elif value == "o" :
                self.open_rakes_msg.data = True
                publisherID=2
            elif value == "c" :
                self.open_rakes_msg.data = False
                publisherID=2

            if publisherID == 1 :
                self.get_logger().info(
                    f"goto {self.goto_msg.pose.position.x} {self.goto_msg.pose.position.y} {self.goto_msg.pose.position.z}"
                )
                now = self.get_clock().now().to_msg()
                self.goto_msg.header.stamp = now
                self.pub_goto.publish(self.goto_msg)

            elif publisherID == 2 :
                self.get_logger().info(
                    f"opening rakes" if self.open_rakes_msg.data else "closing rakes"
                )
                self.pub_rakes.publish(self.open_rakes_msg)


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
