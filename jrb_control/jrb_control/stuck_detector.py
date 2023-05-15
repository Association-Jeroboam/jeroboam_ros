import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray
from geometry_msgs.msg import Twist
from rclpy.time import Time, Duration
from nav_msgs.msg import Odometry
from math import exp


# TODO : add proper timestamp on can_bridge can rx
# TODO : twist stamped instead of twist
class StuckDetector(Node):
    def __init__(self):
        super().__init__("stuck_detector")
        self.wheel_base = 0.26085  # m
        self.min_wheel_velocity = 0.01  # m/s
        self.max_wheel_velocity = 1.00  # m/s

        self.charge_time = 0.25  # s
        self.tau = self.charge_time / 3.0  # s
        self.frequency = 20  # hz
        self.dt = 1 / self.frequency
        self.alpha = 1 - exp(-self.dt / self.tau)
        self.error_threshold = 0.95  # attained at 3 tau = charge_time

        self.odom_msg = None
        self.cmd_vel_msg = None

        self.last_compute_time = None
        self.error_mean_avg = 0.0

        self._sub_odometry = self.create_subscription(
            Odometry, "/odometry", self.odom_callback, 10
        )
        self._sub_cmd_vel = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_vel_callback, 10
        )

        self.pub_stuck = self.create_publisher(Bool, "/stuck", 10)
        self.pub_debug = self.create_publisher(
            Float32MultiArray, "/debug/stuck_error", 10
        )

        self.filter_timer = self.create_timer(
            self.dt, lambda: self.compute_filter(self.odom_msg, self.cmd_vel_msg)
        )

    def compute_v_wheel_from_twist(self, twist_msg: Twist):
        v_right = twist_msg.linear.x + twist_msg.angular.z * (self.wheel_base / 2)
        v_left = twist_msg.linear.x - twist_msg.angular.z * (self.wheel_base / 2)
        return (v_right, v_left)

    def odom_callback(self, msg):
        self.odom_msg = msg

    def cmd_vel_callback(self, msg):
        self.cmd_vel_msg = msg

    def compute_filter(self, odom_msg: Odometry, cmd_vel_msg: Twist):
        # Wait for at least a couple of message to be received
        if odom_msg is None or cmd_vel_msg is None:
            return

        # Check if messages aren't too old. Max accepted delay is 0.25sec
        now = self.get_clock().now()
        max_time = now - Duration(seconds=0.25)

        odom_time = Time.from_msg(odom_msg.header.stamp)
        if odom_time < max_time:
            self.get_logger().info("odom is too old")
            return

        # TODO : same with twist stamped
        # TODO : check desync between the two messages

        v_right, v_left = self.compute_v_wheel_from_twist(odom_msg.twist.twist)
        v_right_cmd, v_left_cmd = self.compute_v_wheel_from_twist(cmd_vel_msg)

        # Compute normalized errors, avoiding division by zero
        if abs(v_right_cmd) > 0.1:
            v_right_error = abs((v_right_cmd - v_right) / v_right_cmd)
        else:
            v_right_error = 0.0

        if abs(v_left_cmd) > 0.1:
            v_left_error = abs((v_left_cmd - v_left) / v_left_cmd)
        else:
            v_left_error = 0.0

        # We want the errors to be in the range [0, 1] as we only measure undershoot, not overshoot
        v_right_error = 0.0 if v_right_error > 1 else v_right_error
        v_left_error = 0.0 if v_left_error > 1 else v_left_error

        error_mean = 0.5 * (v_right_error + v_left_error)

        # Init filter or reset if last compute is too old
        if not self.last_compute_time is not None or self.last_compute_time < max_time:
            self.get_logger().info("reset filter")
            self.error_mean_avg = error_mean
        else:
            # Accumulate the error mean in a moving average (low pass filter)
            self.error_mean_avg += self.alpha * (error_mean - self.error_mean_avg)

        # debug_msg = Float32MultiArray()
        # debug_msg.data = [
        #     v_right,
        #     v_left,
        #     v_right_cmd,
        #     v_left_cmd,
        #     v_right_error,
        #     v_left_error,
        #     error_mean,
        #     self.error_mean_avg,
        # ]
        # self.pub_debug.publish(debug_msg)

        if self.error_mean_avg > self.error_threshold:
            self.get_logger().info("Robot stuck !")
            stuck_msg = Bool(data=True)
            self.pub_stuck.publish(stuck_msg)

        self.last_compute_time = now


def main(args=None):
    rclpy.init(args=args)

    stuck_detector = StuckDetector()

    rclpy.spin(stuck_detector)

    stuck_detector.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
