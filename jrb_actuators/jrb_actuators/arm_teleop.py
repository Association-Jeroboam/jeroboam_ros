#!/usr/bin/env python3

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
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from jrb_msgs.action import GoToPose
import math

import time


class ArmTeleop(Node):
    def __init__(self):
        super().__init__("arm_teleop")
        self.get_logger().info("init")

        self.left_action_goto = ActionClient(self,GoToPose,'left_arm_go_to_goal')
        self.right_action_goto = ActionClient(self,GoToPose,'right_arm_go_to_goal')

        #self.pub_left_goto = self.create_publisher(PoseStamped, "left_arm_goto", 10)
        #self.pub_right_goto = self.create_publisher(PoseStamped, "right_arm_goto", 10)

        self.pub_rakes = self.create_publisher(Bool, "open_rakes",10)

        self.pub_left_pump = self.create_publisher(Bool, "pump_left",10)
        self.pub_right_pump = self.create_publisher(Bool, "pump_right",10)



        self.goto_msg = GoToPose.Goal()
        self.goto_msg.pose.pose.position.x = 0.0
        self.goto_msg.pose.pose.position.y = 0.13
        self.goto_msg.pose.pose.position.z = 0.22

        self.goto_msg.pose.pose.orientation.x = math.radians(90)
        self.goto_msg.pose.pose.orientation.y = 0.0
        self.goto_msg.pose.pose.orientation.z = 0.0
        self.goto_msg.pose.pose.orientation.w = 0.0

        self.open_rakes_msg = Bool()

        self.pump_msg = Bool()

        self.side="left"
        self.goto_msg.pose.header.frame_id = "left_arm_origin_link"


    def loop(self):
        self.get_logger().info("Press any key to continue! (or press ESC to quit!)")
        while rclpy.ok():
            publisherID=1
            value = getch()
            if value == chr(0x1B):
                break

            elif value == "1" :
                if self.side == "left" :
                    self.side = "right"
                    self.goto_msg.pose.header.frame_id = "right_arm_origin_link" 
                    
                else :
                    self.side = "left"
                    self.goto_msg.pose.header.frame_id = "left_arm_origin_link" 
                print("Selected arm is :",self.side)
                continue

            #bras :
            elif value == "q":
                self.goto_msg.pose.pose.position.x -= 0.005
            elif value == "d":
                self.goto_msg.pose.pose.position.x += 0.005
            elif value == "z":
                self.goto_msg.pose.pose.position.y += 0.005
            elif value == "s":
                self.goto_msg.pose.pose.position.y -= 0.005
            elif value == "a":
                self.goto_msg.pose.pose.position.z -= 0.005
            elif value == "e":
                self.goto_msg.pose.pose.position.z += 0.005
                
            elif value == "h": #homing
                self.goto_msg.pose.pose.position.x = 0.0
                self.goto_msg.pose.pose.position.y = 0.13
                self.goto_msg.pose.pose.position.z = 0.100
                self.goto_msg.pose.pose.orientation.x = math.radians(90)
                self.goto_msg.pose.pose.orientation.y = 0.0
                self.goto_msg.pose.pose.orientation.z = 0.0

            #poignet
            elif value == "6":
                self.goto_msg.pose.pose.orientation.x -= math.radians(5)
            elif value == "4":
                self.goto_msg.pose.pose.orientation.x += math.radians(5)
            elif value == "8":
                self.goto_msg.pose.pose.orientation.y -= math.radians(5)
            elif value == "5":
                self.goto_msg.pose.pose.orientation.y += math.radians(5)
            elif value == "9":
                self.goto_msg.pose.pose.orientation.z -= math.radians(5)
            elif value == "7":
                self.goto_msg.pose.pose.orientation.z += math.radians(5)

            #rateaux
            elif value == "o" :
                self.open_rakes_msg.data = True
                publisherID=2
            elif value == "c" :
                self.open_rakes_msg.data = False
                publisherID=2

            #pompe
            elif value == "2" :
                self.pump_msg.data = True
                publisherID=3
            elif value == "3" :
                self.pump_msg.data = False
                publisherID=3


            if publisherID == 1 :
                self.get_logger().info(
                    f"goto ({self.goto_msg.pose.pose.position.x} {self.goto_msg.pose.pose.position.y} {self.goto_msg.pose.pose.position.z}) ({self.goto_msg.pose.pose.orientation.y}, {self.goto_msg.pose.pose.orientation.x}, {self.goto_msg.pose.pose.orientation.z})"
                )
                now = self.get_clock().now().to_msg()
                self.goto_msg.pose.header.stamp = now

                if self.side == "left" :
                #    self.pub_left_goto.publish(self.goto_msg)
                    if not self.left_action_goto.server_is_ready() :
                        self.get_logger().warn(f"Waiting for an action server ({self.left_action_goto._action_name}) to become available...")
                        self.left_action_goto .wait_for_server()
                        self.get_logger().info(f"Action server {self.left_action_goto._action_name} find !")
                    self.left_action_goto.send_goal_async(self.goto_msg)
                else :
                #    self.pub_right_goto.publish(self.goto_msg)
                    if not self.right_action_goto.server_is_ready() :
                        self.get_logger().warn(f"Waiting for an action server ({self.right_action_goto._action_name}) to become available...")
                        self.right_action_goto .wait_for_server()
                        self.get_logger().info(f"Action server {self.right_action_goto._action_name} find !")
                    self.right_action_goto.send_goal_async(self.goto_msg)

            elif publisherID == 2 :
                self.get_logger().info(
                    f"opening rakes" if self.open_rakes_msg.data else "closing rakes"
                )
                self.pub_rakes.publish(self.open_rakes_msg)

            elif publisherID == 3 :
                self.get_logger().info(
                    f"Starting pump" if self.pump_msg.data else "Stopping pump"
                )
                if self.side == "left" :
                    self.pub_left_pump.publish(self.pump_msg)
                else :
                    self.pub_right_pump.publish(self.pump_msg)                    

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
