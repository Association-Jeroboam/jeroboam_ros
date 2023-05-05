#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import subprocess
import signal
import os
from pathlib import Path

class BagPlayer(Node):
    def __init__(self):
        super().__init__('bag_player')
        self.process = None
        self.bag_dir = Path(os.path.dirname(os.path.realpath(__file__)) / Path("../bags/")).resolve()


        self.start_subscriber = self.create_subscription(
            String,
            '/start_bag_playback',
            self.start_bag_playback_callback,
            10
        )

    def start_bag_playback_callback(self, msg):
        if not msg.data:
            return
        
        self.bag_name = msg.data
        self.bag_path = str((Path(self.bag_dir) / Path(self.bag_name)).resolve())
        self.start_bag_playback()

    def start_bag_playback(self):
        if not os.path.exists(self.bag_path):
            self.get_logger().error(f"Bag file not found: {self.bag_path}")
            return

        if self.process is None or self.process.poll() is not None:
            command = ['ros2', 'bag', 'play', self.bag_path]
            self.process = subprocess.Popen(command)
            self.get_logger().info(f"Started playing bag: {self.bag_name}")
        else:
            self.get_logger().warn("Bag is already playing")

    def stop_bag_playback(self):
        if self.process is not None:
            self.process.terminate()
            self.process = None
            self.get_logger().info(f"Stopped playing bag: {self.bag_name}")

def main(args=None):
    rclpy.init(args=args)

    bag_player = BagPlayer()

    try:
        rclpy.spin(bag_player)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        bag_player.destroy_node()

if __name__ == "__main__":
    main()