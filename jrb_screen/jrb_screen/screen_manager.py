import os

from pyray import *
import math
import traceback

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Empty
import threading
from ament_index_python import get_package_share_directory
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, IntegerRange
from rcl_interfaces.msg import SetParametersResult


DATA_PATH = get_package_share_directory("jrb_screen")

STATE_WAIT_FOR_MATCH = 0
STATE_MATCH = 1
STATE_AFTER_MATCH = 2


class ScreenManager(Node):
    def __init__(self):
        super().__init__("screen_manager")
        self.get_logger().info("init")

        self.declare_parameter(
            "width",
            800,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                integer_range=[IntegerRange(from_value=100, to_value=1000, step=10)],
                read_only=True,
            ),
        )

        self.declare_parameter(
            "height",
            480,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                integer_range=[IntegerRange(from_value=100, to_value=1000, step=10)],
                read_only=True,
            ),
        )

        self.declare_parameter(
            "fps",
            5,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                integer_range=[IntegerRange(from_value=1, to_value=60, step=1)],
                read_only=True,
            ),
        )

        latchedQoS = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=1,
        )

        self.sub_sample_detected = self.create_subscription(
            String, "screen/score", self.on_score, 10
        )

        self.sub_sample_detected = self.create_subscription(
            String, "screen/text", self.on_text, 10
        )

        self.sub_team = self.create_subscription(
            Bool, "hardware/starter", self.on_starter, latchedQoS
        )

        self.sub_sample_detected = self.create_subscription(
            String, "hardware/team", self.on_team, latchedQoS
        )

        self.sub_sample_detected = self.create_subscription(
            Bool, "hardware/strategy", self.on_strategy, latchedQoS
        )

        self.sub_sample_detected = self.create_subscription(
            Empty, "strategy/end_match", self.on_end_match, 1
        )

        self.width = self.get_parameter("width").value
        self.height = self.get_parameter("height").value
        self.fps = self.get_parameter("fps").value

        self.state = STATE_WAIT_FOR_MATCH
        self.stop = False
        self.score = "0"
        self.text = ""
        self.team = ""
        self.strategy = ""

        self.raylib_thread = threading.Thread(target=self.loop, daemon=True)
        self.raylib_thread.start()

        self.add_on_set_parameters_callback(self.on_update_parameters)

    def on_update_parameters(self, params):
        pass
        self.get_logger().info("Params updated")

        return SetParametersResult(successful=True)

    def __del__(self):
        if hasattr(self, "raylib_thread") and self.raylib_thread is not None:
            self.raylib_thread.join()

    def on_score(self, msg: String):
        self.score = msg.data

    def on_text(self, msg: String):
        self.text = msg.data

    def on_starter(self, msg: Bool):
        if self.state == STATE_WAIT_FOR_MATCH and msg.data == True:
            self.state = STATE_MATCH
            return

        if self.state == STATE_AFTER_MATCH and msg.data == False:
            self.state = STATE_WAIT_FOR_MATCH
            return

    def on_team(self, msg: String):
        self.team = msg.data

    def on_strategy(self, msg: Bool):
        self.strategy = str(msg.data)

    def on_end_match(self, msg: Empty):
        self.state = STATE_AFTER_MATCH

    def start_window(self):
        pass

    def loop(self):
        init_window(self.width, self.height, "raylib [core] example - basic window")
        set_target_fps(self.fps)

        image = load_image(os.path.join(DATA_PATH, "assets/Jeroboam.png"))
        image_resize(image, 200, 200)
        logo_texture = load_texture_from_image(image)
        unload_image(image)

        while rclpy.ok() and not window_should_close():
            begin_drawing()
            clear_background(RAYWHITE)
            draw_texture(
                logo_texture,
                10,
                10,
                WHITE,
            )

            if self.state == STATE_WAIT_FOR_MATCH:
                big_text = "Le match va commencer..."
                subtext = f"Équipe: {self.team} Stratégie: {self.strategy}"
            elif self.state == STATE_MATCH:
                big_text = f"SCORE: {self.score}"
                subtext = ""
            elif self.state == STATE_AFTER_MATCH:
                big_text = f"SCORE: {self.score}"
                subtext = "Match fini !"

            big_text_width = measure_text(big_text, 50)
            subtext_width = measure_text(subtext, 20)
            text_width = measure_text(self.text, 20)

            draw_text(
                big_text,
                round((self.width - big_text_width) / 2),
                round(0.45 * self.height),
                50,
                MAROON,
            )

            draw_text(
                subtext,
                round((self.width - subtext_width) / 2),
                round(0.7 * self.height),
                20,
                LIGHTGRAY,
            )

            draw_text(
                self.text,
                round((self.width - text_width) / 2),
                round(0.8 * self.height),
                20,
                LIGHTGRAY,
            )

            end_drawing()

        close_window()


def main(args=None):
    rclpy.init(args=args)

    node = ScreenManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped cleanly")
    except Exception:
        print("Error while stopping the node")
        print(traceback.format_exc())
        raise
    finally:
        rclpy.shutdown()
        node.destroy_node()


if __name__ == "__main__":
    main()
