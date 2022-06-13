import traceback
import threading
import code

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from math import radians, degrees


class PointRecorderNode(Node):
    def __init__(self):
        super().__init__("point_recorder")
        self.get_logger().info("init")

        self.pub_initialpose = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self.sub_odometry = self.create_subscription(
            Odometry, "/odometry", self.on_odometry, 10
        )

        self.current_pose = None

    def on_odometry(self, msg: Odometry):
        self.current_pose = msg

    def pose(self):
        q = [
            self.current_pose.pose.pose.orientation.x,
            self.current_pose.pose.pose.orientation.y,
            self.current_pose.pose.pose.orientation.z,
            self.current_pose.pose.pose.orientation.w,
        ]
        _, _, theta = euler_from_quaternion(q)
        x = self.current_pose.pose.pose.position.x
        y = self.current_pose.pose.pose.position.y

        print(f"self.goto({x}, {y}, radians({degrees(theta)}))")

    def set_pose(self, theta, x=None, y=None):
        q = quaternion_from_euler(0, 0, radians(theta))

        pose_msg = PoseWithCovarianceStamped()

        pose_msg.header.stamp = self.get_clock().now().to_msg()

        currentX = self.current_pose.pose.pose.position.x
        currentY = self.current_pose.pose.pose.position.y

        pose_msg.pose.pose.position.x = float(x) if x is not None else currentX
        pose_msg.pose.pose.position.y = float(y) if y is not None else currentY

        pose_msg.pose.pose.orientation.x = q[0]
        pose_msg.pose.pose.orientation.y = q[1]
        pose_msg.pose.pose.orientation.z = q[2]
        pose_msg.pose.pose.orientation.w = q[3]

        self.pub_initialpose.publish(pose_msg)

        print(f"self.recalibration({theta}, x={x}, y={y})")


def main(args=None):
    rclpy.init(args=args)

    node = PointRecorderNode()

    # Spin rclpy on separate thread
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    try:
        code.InteractiveConsole(
            locals={"pose": node.pose, "set_pose": node.set_pose}
        ).interact()
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped cleanly")
    except Exception:
        print("Error while stopping the node")
        print(traceback.format_exc())
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()
        thread.join()


if __name__ == "__main__":
    main()
