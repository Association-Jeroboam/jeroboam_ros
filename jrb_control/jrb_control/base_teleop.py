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
from geometry_msgs.msg import Twist
from rclpy.time import Time, Duration
from math import radians
import threading


class BaseTeleop(Node):
    def __init__(self):
        super().__init__("base_teleop")
        self.get_logger().info("init")

        self.pub_cmd_vel = self.create_publisher(Twist, "cmd_vel", 10)

        self.timeout = 0.25  # secs
        rate_value = 30.0  # Hz
        self.dT = 1 / rate_value  # secs
        self.rate = self.create_rate(rate_value)
        self.linear_acceleration = 0.5
        self.angular_acceleration = radians(180)

        self.v = 0.0
        self.omega = 0.0
        self.last_command = self.get_clock().now()

        self.cmd_vel_msg = Twist()

        self.publish_thread = threading.Thread(target=self.publish_loop, daemon=True)
        self.publish_thread.start()

    def __del__(self):
        self.publish_thread.join()

    def publish_loop(self):
        while rclpy.ok():
            now = self.get_clock().now()

            if now > (self.last_command + Duration(seconds=self.timeout)):
                self.v = 0.0
                self.omega = 0.0
                self.last_command = now

            self.cmd_vel_msg.linear.x = self.v
            self.cmd_vel_msg.angular.z = self.omega
            self.pub_cmd_vel.publish(self.cmd_vel_msg)
            self.rate.sleep()

    def loop(self):
        self.get_logger().info("Press any key to continue! (or press ESC to quit!)")

        while rclpy.ok():
            value = getch()

            if value == chr(0x1B):
                break
            elif value == "q":
                self.omega += self.angular_acceleration * self.dT
            elif value == "d":
                self.omega -= self.angular_acceleration * self.dT
            elif value == "z":
                self.v = self.v + self.linear_acceleration * self.dT
            elif value == "s":
                self.v -= self.linear_acceleration * self.dT

            self.last_command = self.get_clock().now()
            print(self.v, self.omega)

        self.get_logger().info("Node stopped cleanly")


def main(args=None):
    rclpy.init(args=args)
    node = BaseTeleop()

    # Spin rclpy on separate thread
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

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
