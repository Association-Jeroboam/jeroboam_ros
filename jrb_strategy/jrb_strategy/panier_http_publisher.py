#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import sys
import subprocess
import signal
import os
from pathlib import Path
import requests
from std_msgs.msg import UInt8

PANIER_URL = "http://192.168.1.164:8000/counter"
PULLING_PERIOD_S = 1

class PanierHttpPublisher(Node):
    def __init__(self):
        super().__init__('panier_http_publisher')
        self.score_publisher = self.create_publisher(UInt8, "/panier/score_http", 10)
        self.timer = self.create_timer(PULLING_PERIOD_S, self.timer_callback)

    def timer_callback(self):
        try:
            response = requests.get(PANIER_URL)
            response.raise_for_status()
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Failed to request panier score: {e}")
            return

        counter = int(response.text)
        if counter < 0:
            self.get_logger().error(f"Panier is not ready (counter = -1)")
            return

        score_msg = UInt8()
        score_msg.data = int(counter)
        self.score_publisher.publish(score_msg)


def main(args=None):
    rclpy.init(args=args)
    panier_http_publisher = PanierHttpPublisher()

    try:
        rclpy.spin(panier_http_publisher)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        panier_http_publisher.destroy_node()

if __name__ == "__main__":
    main()
