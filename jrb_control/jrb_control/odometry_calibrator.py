import sys
import time
from math import pi, atan2, degrees
import rclpy
from tf_transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from jrb_msgs.msg import OdometryTicks
import traceback
from time import process_time

DEFAULT_DISTANCE = 0.1335
NUMBER_OF_ROTATIONS = 10
LINEAR_VELOCITY = 0.25
ANGULAR_VELOCITY = 2.0
DIRECTION = -1

class OdometryCalibratorNode(Node):
    def __init__(self):
        super().__init__("odometry_calibrator")

        # Internal state
        self.odom_angular_last = 0.0
        self.odom_angular_last_abs = 0.0
        self.odom_angular_start = 0.0
        self.odom_linear_start = 0.0
        self.odom_params_initialised = False
        self.calib_finished = False

        self.twist_msg:Twist = Twist()

        self.left = 0
        self.right = 0
        self.yaw = 0.0
        self.error = 0.0

        # Parameters
        self.type = self.declare_parameter('type', 'angular')
        self.distance = self.declare_parameter('distance', DEFAULT_DISTANCE)

        if self.type.value not in ['angular', 'linear']:
            self.get_logger().warn(f"Invalid parameter value. The value of 'type' should be one of 'angular' or 'linear'. Setting it to 'angular'.")
            self.set_parameters([Parameter('type', Parameter.Type.STRING, 'angular')])
            self.type = self.get_parameter('type')

        # Subscribers
        self.create_subscription(Odometry, '/odometry', self.odometry_callback, 10)
        # self.create_subscription(OdometryTicks, '/hardware/base/odometry_ticks', self.ticks_callback, 1)

        # Publishers
        self.pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        # Timers
        self.timer = self.create_timer(1, self.timer_callback)

        self.get_logger().info("odometry_calibrator initialized with type: " + self.type.value)

    def set_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'  # Usually the frame is 'map'
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 1.0  # Quaternion for zero rotation

        self.publisher.publish(msg)
        self.get_logger().info('Initial pose set to 0,0,0')

    def ticks_callback(self, msg:OdometryTicks):
        self.left += msg.left
        self.right += msg.right

    def timer_callback(self):
        # self.get_logger().info(str(self.left) + " " + str(self.right))
        # self.get_logger().info(str(degrees(self.yaw)))
        self.get_logger().info(f'estimated error: {self.error:.6f}, yaw: {str(degrees(self.yaw))}')


    def finish_calibration(self):
        self.calib_finished = True
        self.set_velocity(0, 0)
        # self.get_logger().info('The robot has reached the given pose according to odometry')
        # time.sleep(0.5)
        # self.destroy_node()
        # sys.exit(0)

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
        t1_start = process_time() 

        q = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ]
        _, _, self.yaw = euler_from_quaternion(q)

        if not self.odom_params_initialised:
            if self.yaw < 0:
                self.yaw = 2 * pi + self.yaw

            self.odom_params_initialised = True
            self.odom_angular_last = self.yaw
            self.odom_angular_start = self.yaw
            self.odom_angular_last_abs = self.yaw
            self.odom_linear_start = msg.pose.pose.position.x

            return

        # Resolve singularities (first for positive angle and then for negative angle)
        if self.yaw - self.odom_angular_last > pi:
            self.odom_angular_last_abs += 2*pi - (self.yaw - self.odom_angular_last)
        elif self.odom_angular_last - self.yaw > pi:
            self.odom_angular_last_abs += 2*pi - (self.odom_angular_last - self.yaw)
        else:
            self.odom_angular_last_abs += self.yaw - self.odom_angular_last

        # Angular calibration
        if self.type.value == 'angular':
            # self.get_logger().info('Angular calibration in progress...')
            if not self.calib_finished:
                self.set_velocity(0, DIRECTION*ANGULAR_VELOCITY)

            n_rotations = (self.odom_angular_last_abs - self.odom_angular_start)/(2*pi)
            if abs(n_rotations) > NUMBER_OF_ROTATIONS:
                self.finish_calibration()

            # if self.calib_finished:
            self.error = DIRECTION*360.0 * (n_rotations - DIRECTION*NUMBER_OF_ROTATIONS)
            # self.get_logger().info(f'estimated error: {error:.6f}')
            t1_stop = process_time()
        else:
            # Linear calibration
            # self.get_logger().info('Linear calibration in progress...')
            self.set_velocity(LINEAR_VELOCITY, 0)
            passed_distance = msg.pose.pose.position.x - self.odom_linear_start
            if passed_distance > self.distance.value:
                # self.get_logger().info(f'Passed distance: {passed_distance:.4f}')
                self.finish_calibration()

        # Save readings
        self.odom_angular_last = self.yaw
        # print(str(t1_stop-t1_start))


def main(args=None):
    rclpy.init(args=args)

    node = OdometryCalibratorNode()
    node.set_initial_pose()

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