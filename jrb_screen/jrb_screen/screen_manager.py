import os

from pyray import *
import traceback
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
from ament_index_python import get_package_share_directory


DATA_PATH = get_package_share_directory("jrb_screen")


class ScreenManager(Node):
    def __init__(self):
        super().__init__("screen_manager")
        self.get_logger().info("init")

        self.sub_sample_detected = self.create_subscription(
            String, "screen/score", self.on_score, 10
        )

        self.sub_sample_detected = self.create_subscription(
            String, "screen/text", self.on_text, 10
        )

        self.raylib_thread = threading.Thread(target=self.loop, daemon=True)
        self.raylib_thread.start()

        self.stop = False
        self.score = "0"
        self.text = "Le match va commencer..."

    def cleanup(self):
        if self.raylib_thread:
            self.raylib_thread.join()

    def on_score(self, msg: String):
        self.score = msg.data

    def on_text(self, msg: String):
        self.text = msg.data

    def loop(self):
        width = 800
        height = 450
        init_window(width, height, "raylib [core] example - basic window")

        image = load_image(os.path.join(DATA_PATH, "assets/Jeroboam.png"))
        image_resize(image, 200, 200)
        texture = load_texture_from_image(image)
        unload_image(image)

        set_target_fps(5)

        while rclpy.ok() and not window_should_close():
            begin_drawing()
            clear_background(RAYWHITE)
            draw_texture(
                texture,
                10,
                10,
                WHITE,
            )
            draw_text(f"SCORE: {self.score}", 250, 200, 50, MAROON)
            draw_text(self.text, 250, 310, 20, LIGHTGRAY)
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
        node.cleanup()
        node.destroy_node()


if __name__ == "__main__":
    main()
