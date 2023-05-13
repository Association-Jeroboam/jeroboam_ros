import sys
import time
from math import pi, atan2
import rclpy
from tf_transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
import traceback

DEFAULT_DISTANCE = 0.1335
NUMBER_OF_ROTATIONS = 10
LINEAR_VELOCITY = 0.25
ANGULAR_VELOCITY = 2.0

class OdometryCalibratorNode(Node):
    def __init__(self):
        super().__init__("odometry_calibrator")

        # Parameters
        self.type = self.declare_parameter('type', 'angular')
        self.distance = self.declare_parameter('distance', DEFAULT_DISTANCE)

        if self.type.value not in ['angular', 'linear']:
            self.get_logger().warn(f"Invalid parameter value. The value of 'type' should be one of 'angular' or 'linear'. Setting it to 'angular'.")
            self.set_parameters([Parameter('type', Parameter.Type.STRING, 'angular')])
            self.type = self.get_parameter('type')

        # Subscribers
        self.create_subscription(Odometry, '/odom', self.odometry_callback, 1)

        # Publishers
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Internal state
        self.odom_angular_last = 0.0
        self.odom_angular_last_abs = 0.0
        self.odom_angular_start = 0.0
        self.odom_linear_start = 0.0
        self.odom_params_initialised = False

        self.twist_msg:Twist = Twist()

        self.get_logger().info("odometry_calibrator initialized with type: " + self.type.value)

    def finish_calibration(self):
        self.set_velocity(0, 0)
        self.get_logger().info('The robot has reached the given pose according to odometry')
        time.sleep(0.5)
        self.destroy_node()
        sys.exit(0)

    def set_velocity(self, linear:float, angular:float):
        self.twist_msg = Twist()
        self.twist_msg.angular.x = 0.0
        self.twist_msg.angular.y = 0.0
        self.twist_msg.angular.z = float(angular)
        self.twist_msg.linear.x = float(linear)
        self.twist_msg.linear.y = 0.0
        self.twist_msg.linear.z = 0.0

        self.pub.publish(self.twist_msg)

    def odometry_callback(self, msg: Odometry):
        q = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ]
        _, _, yaw = euler_from_quaternion(q)

        if not self.odom_params_initialised:
            if yaw < 0:
                yaw = 2 * pi + yaw

            self.odom_params_initialised = True
            self.odom_angular_last = yaw
            self.odom_angular_start = yaw
            self.odom_angular_last_abs = yaw
            self.odom_linear_start = msg.pose.pose.position.x

            return

        # Resolve singularities (first for positive angle and then for negative angle)
        if yaw - self.odom_angular_last > pi:
            self.odom_angular_last_abs += 2*pi - (yaw - self.odom_angular_last)
        elif self.odom_angular_last - yaw > pi:
            self.odom_angular_last_abs += 2*pi - (self.odom_angular_last - yaw)
        else:
            self.odom_angular_last_abs += yaw - self.odom_angular_last

        # Angular calibration
        if self.type.value == 'angular':
            self.get_logger().info('Angular calibration in progress...')
            self.set_velocity(0, ANGULAR_VELOCITY)
            n_rotations = (self.odom_angular_last_abs - self.odom_angular_start)/(2*pi)
            if n_rotations > NUMBER_OF_ROTATIONS:
                self.finish_calibration()
            self.get_logger().info(f'Number of rotations: {n_rotations:.4f}')

        # Linear calibration
        if self.type.value == 'linear':
            self.get_logger().info('Linear calibration in progress...')
            self.set_velocity(LINEAR_VELOCITY, 0)
            passed_distance = msg.pose.pose.position.x - self.odom_linear_start
            self.get_logger().info(f'Passed distance: {passed_distance:.4f}')
            if passed_distance > self.distance.value:
                self.finish_calibration()

        # Save readings
        self.odom_angular_last = yaw


def main(args=None):
    rclpy.init(args=args)

    node = OdometryCalibratorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Node stopped cleanly")
    except Exception:
        print("Error while stopping the node")
        print(traceback.format_exc())
        raise

if __name__ == "__main__":
    main()