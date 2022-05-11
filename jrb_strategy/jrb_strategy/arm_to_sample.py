import traceback
import rclpy
from rclpy.node import Node

from jrb_msgs.msg import SampleDetectedArray
from geometry_msgs.msg import PoseStamped


class ArmToSample(Node):
    def __init__(self):
        super().__init__("sample_detector")
        self.get_logger().info("init")

        self.pub_left_arm_goto = self.create_publisher(
            PoseStamped, "/left_arm_goto", 10
        )
        self.sub_sample_detected = self.create_subscription(
            SampleDetectedArray, "sample_detected", self.on_sample_detected, 10
        )

    def on_sample_detected(self, msg: SampleDetectedArray):
        if len(msg.samples) == 0:
            return

        poses = [sample.pose for sample in msg.samples]

        left_arm_goal = PoseStamped()
        left_arm_goal.header = msg.header
        left_arm_goal.pose = poses[0]

        self.get_logger().info(
            f"Sample detected ! left arm GOTO {left_arm_goal.pose.position.x} {left_arm_goal.pose.position.y} {left_arm_goal.pose.position.z}"
        )

        self.pub_left_arm_goto.publish(left_arm_goal)


def main(args=None):
    rclpy.init(args=args)

    node = ArmToSample()

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


if __name__ == "__main__":
    main()
