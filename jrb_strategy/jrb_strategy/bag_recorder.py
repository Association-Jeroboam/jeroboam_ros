#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import sys
import subprocess
import signal
import os
from pathlib import Path

class BagRecorder(Node):
    def __init__(self, bag_name):
        super().__init__('bag_recorder')
        self.bag_name = bag_name
        self.bag_dir = Path(os.path.dirname(os.path.realpath(__file__)) / Path("../bags/")).resolve()
        self.bag_path = str((Path(self.bag_dir) / Path(self.bag_name)).resolve())
        self.process = None

    def record_bag(self):
        topics = [
            '/cmd_vel', 
            '/hardware/arduino/serial_write', 
            '/hardware/turbine/speed',
            '/hardware/led',
            '/hardware/roll/height'
        ]
        command = ['ros2', 'bag', 'record', '-o', self.bag_path] + topics

        self.process = subprocess.Popen(command)
        self.get_logger().info(f"Started recording bag: {self.bag_name}")

    def stop_bag_recording(self):
        if self.process is not None:
            self.process.send_signal(signal.SIGINT)

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print("Please provide a bag name as a CLI argument.")
        sys.exit(1)

    bag_name = sys.argv[1]
    bag_recorder = BagRecorder(bag_name)

    try:
        bag_recorder.record_bag()
        rclpy.spin(bag_recorder)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        bag_recorder.stop_bag_recording()
        bag_recorder.destroy_node()

if __name__ == "__main__":
    main()