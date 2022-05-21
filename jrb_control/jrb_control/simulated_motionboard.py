import traceback

from rcl_interfaces.msg import ParameterDescriptor, ParameterType, FloatingPointRange
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from math import cos, sin
from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import threading
from tf_transformations import quaternion_from_euler


class SimulatedMotionboard(Node):
    def __init__(self):
        super().__init__("simulated_motionboard")
        self.get_logger().info(f"{self.get_name()} started")

        self.declare_parameter(
            "rate",
            50.0,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                floating_point_range=[
                    FloatingPointRange(from_value=1.0, to_value=200.0, step=1.0)
                ],
            ),
        )

        self.odom_pub = self.create_publisher(Odometry, "odometry", 10)

        self.cmd_vel_sub = self.create_subscription(
            Twist, "cmd_vel", self.on_cmd_vel, 10
        )

        self.initialpose_sub = self.create_subscription(
            PoseWithCovarianceStamped, "/initialpose", self.on_initialpose, 10
        )

        self.v = 0.0
        self.omega = 0.0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        rate_value = self.get_parameter("rate").value
        self.dT = 1 / rate_value
        self.rate = self.create_rate(rate_value)

        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = "odom"
        self.odom_msg.child_frame_id = "base_footprint"

        self.add_on_set_parameters_callback(self.on_update_parameters)

    def on_update_parameters(self, params):
        for param in params:
            if param.name == "rate":
                self.rate = self.create_rate(param.value)
                self.dT = 1 / param.value

        self.get_logger().info("Params updated")

        return SetParametersResult(successful=True)

    def on_cmd_vel(self, msg: Twist):
        self.omega = msg.angular.z
        self.v = msg.linear.x

    def on_initialpose(self, msg: PoseWithCovarianceStamped):
        q = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ]
        _, _, self.theta = euler_from_quaternion(q)
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        self.get_logger().info(f"Position set to ({self.x}, {self.y}, ${self.theta})")

    def publish_odometry(self):
        now = self.get_clock().now().to_msg()

        q = quaternion_from_euler(0, 0, self.theta)

        self.odom_msg.header.stamp = now
        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        self.odom_msg.pose.pose.orientation.x = q[0]
        self.odom_msg.pose.pose.orientation.y = q[1]
        self.odom_msg.pose.pose.orientation.z = q[2]
        self.odom_msg.pose.pose.orientation.w = q[3]

        self.odom_pub.publish(self.odom_msg)

    def loop(self):
        while rclpy.ok():
            self.theta += self.dT * self.omega
            self.x += self.dT * self.v * cos(self.theta)
            self.y += self.dT * self.v * sin(self.theta)
            self.publish_odometry()
            self.rate.sleep()


def main(args=None):
    rclpy.init(args=args)

    node = SimulatedMotionboard()

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
        rclpy.shutdown()
        # node.destroy_node()


if __name__ == "__main__":
    main()
