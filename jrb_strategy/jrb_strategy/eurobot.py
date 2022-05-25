import traceback
import threading
import time
from math import pi, atan2, radians

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from rclpy.task import Future

from std_msgs.msg import Bool, String, Empty
from geometry_msgs.msg import PoseStamped, PoseArray, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

from tf_transformations import quaternion_from_euler, euler_from_quaternion

from jrb_msgs.msg import SampleDetectedArray
from jrb_msgs.action import GoToPose


class EurobotStrategyNode(Node):
    def __init__(self):
        super().__init__("eurobot_strategy")
        self.get_logger().info("init")

        latchedQoS = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=1,
        )

        self.cb_group = ReentrantCallbackGroup()

        self.pub_left_arm_goto = self.create_publisher(
            PoseStamped, "/left_arm_goto", 10
        )

        self.pub_right_arm_goto = self.create_publisher(
            PoseStamped, "/right_arm_goto", 10
        )

        self.pub_end_match = self.create_publisher(Empty, "strategy/end_match", 1)

        self.pub_initialpose = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )

        self.goto_action_client = ActionClient(
            self, GoToPose, "diff_drive_go_to_goal", callback_group=self.cb_group
        )

        self.get_logger().info("Wait for GoTo action server...")
        self.goto_action_client.wait_for_server()

        self.goto_goal_msg = GoToPose.Goal()
        self.goto_goal_msg.pose.header.frame_id = "map"  # TODO: dynamic frame_id
        self.goto_goal_handle = None

        self.start = Future()
        self.end_match = Future()
        self.team = None
        self.strategy = None
        self.end_match_timer = None
        self.obstacle_stop = False
        self.currentX = None
        self.currentY = None
        self.currentTheta = None

        # self.sub_sample_detected = self.create_subscription(
        #     SampleDetectedArray, "sample_detected", self.on_sample_detected, 10
        # )

        self.sub_odometry = self.create_subscription(
            Odometry,
            "odometry",
            self.on_odometry,
            10,
            callback_group=self.cb_group,
        )

        self.sub_start = self.create_subscription(
            Bool,
            "hardware/starter",
            self.on_starter,
            latchedQoS,
            callback_group=self.cb_group,
        )
        self.sub_team = self.create_subscription(
            String,
            "hardware/team",
            self.on_team,
            latchedQoS,
            callback_group=self.cb_group,
        )
        self.sub_strategy = self.create_subscription(
            Bool,
            "hardware/strategy",
            self.on_strategy,
            latchedQoS,
            callback_group=self.cb_group,
        )

        self.sub_obstacle = self.create_subscription(
            PoseArray,
            "obstacle_detected",
            self.on_obstacle_detected,
            10,
            callback_group=self.cb_group,
        )

    # def on_sample_detected(self, msg: SampleDetectedArray):
    #     if len(msg.samples) == 0:
    #         return

    #     poses = [sample.pose for sample in msg.samples]

    #     left_arm_goal = PoseStamped()
    #     left_arm_goal.header = msg.header
    #     left_arm_goal.pose = poses[0]

    #     self.get_logger().info(
    #         f"Sample detected ! left arm GOTO {left_arm_goal.pose.position.x} {left_arm_goal.pose.position.y} {left_arm_goal.pose.position.z}"
    #     )

    #     self.pub_left_arm_goto.publish(left_arm_goal)

    def on_starter(self, msg: Bool):
        self.get_logger().info("start cb")
        value = msg.data

        if value:
            self.start.set_result("set")

    def on_team(self, msg: String):
        self.team = msg.data

    def on_strategy(self, msg: Bool):
        self.strategy = msg.data

    def on_obstacle_detected(self, msg: PoseArray):
        if msg.poses.length > 0:
            self.get_logger().warn("Obstacle detected ! Cancelling all goals")
            self.goto_cancel()

    def on_odometry(self, msg: Odometry):
        self.currentX = msg.pose.pose.position.x
        self.currentY = msg.pose.pose.position.y

        q = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ]
        _, _, self.currentTheta = euler_from_quaternion(q)

    def on_end_match(self):
        self.get_logger().info("End match timer")
        self.end_match.set_result("set")

        if self.end_match_timer is not None:
            self.end_match_timer.cancel()

        self.goto_cancel()

    def get_pose(self, x, y, theta):
        if self.team == "yellow":
            return (x, y, theta)
        else:
            return (x, 3.0 - y, 2 * pi - theta)

    def goto_cancel_callback(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info("GoTo goal successfully canceled")
        else:
            self.get_logger().info("GoTo goal failed to cancel")

    def goto_cancel(self):
        self.get_logger().info("GoTo cancel")
        if self.goto_goal_handle is None:
            return

        self.get_logger().warn("Canceling GoTo goal")
        cancel_future = self.goto_goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self.goto_cancel_callback)

        return cancel_future

    def goto_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("GoTo goal rejected :(")
            return

        self.get_logger().info("GoTo goal accepted :)")
        self.goto_goal_handle = goal_handle

    def goto(self, x_, y_, theta_=None):
        if self.end_match.done():
            self.get_logger().warn("Match is finished, ignoring GoTo")
            return False

        if self.obstacle_stop:
            self.get_logger().warn("Obstacle ahead, ignoring GoTo")
            return False

        if theta_ is None:
            self.get_logger().info("No heading provided")

            if self.currentX is None and self.currentY is None:
                self.get_logger().warn(
                    "No odometry data available. Default heading to 0"
                )
                theta_ = 0.0
            else:
                theta_ = atan2(y_ - self.currentY, x_ - self.currentX)

        self.get_logger().info(f"GoTo ({str(x_)}, {str(y_)}, ${str(theta_)}) ")

        self.goto_goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        x, y, theta = self.get_pose(x_, y_, theta_)
        q = quaternion_from_euler(0, 0, theta)

        self.goto_goal_msg.pose.pose.position.x = float(x)
        self.goto_goal_msg.pose.pose.position.y = float(y)
        self.goto_goal_msg.pose.pose.orientation.x = q[0]
        self.goto_goal_msg.pose.pose.orientation.y = q[1]
        self.goto_goal_msg.pose.pose.orientation.z = q[2]
        self.goto_goal_msg.pose.pose.orientation.w = q[3]

        self.goto_action_client.wait_for_server()

        send_goal_future = self.goto_action_client.send_goal_async(self.goto_goal_msg)
        send_goal_future.add_done_callback(self.goto_response_callback)

        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        goal_finished_future = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(self, goal_finished_future)

    def set_initialpose(self, x_, y_, theta_):
        x, y, theta = self.get_pose(x_, y_, theta_)
        q = quaternion_from_euler(0, 0, theta)

        initialpose_msg = PoseWithCovarianceStamped()
        initialpose_msg.header.stamp = self.get_clock().now().to_msg()
        initialpose_msg.header.frame_id = "odom"
        initialpose_msg.pose.pose.position.x = x
        initialpose_msg.pose.pose.position.y = y
        initialpose_msg.pose.pose.orientation.x = q[0]
        initialpose_msg.pose.pose.orientation.y = q[1]
        initialpose_msg.pose.pose.orientation.z = q[2]
        initialpose_msg.pose.pose.orientation.w = q[3]

        self.pub_initialpose.publish(initialpose_msg)

    def loop(self):
        self.get_logger().info("Init strategy. Wait for start...")

        rclpy.spin_until_future_complete(self, self.start)

        self.get_logger().info(
            f"Start ! team: {self.team} strategy: {str(self.strategy)}"
        )
        self.end_match_timer = self.create_timer(
            100, self.on_end_match, callback_group=self.cb_group
        )

        ######### Strategy here, written for YELLOW TEAM #########

        self.set_initialpose(0.865, 0.1, radians(90))

        self.goto(1.118, 0.9441, radians(104.6))

        self.goto(0.69, 1.378, radians(-90))

        self.goto(0.69, 0.3077, radians(-90))

        ######### End strategy ##########

        self.get_logger().info("Strategy finished !")

        rclpy.spin_until_future_complete(self, self.end_match)
        self.get_logger().info("End match !")

        self.rate = self.create_rate(10.0)  # Hz
        while rclpy.ok():
            self.pub_end_match.publish(Empty())
            self.rate.sleep()
            rclpy.spin_once(self)


def main(args=None):
    rclpy.init(args=args)

    node = EurobotStrategyNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)

    thread = None

    try:
        # Spin executor on separate thread
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        node.loop()
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped cleanly")
    except Exception:
        print("Error while stopping the node")
        print(traceback.format_exc())
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()

        if thread is not None:
            thread.join()


if __name__ == "__main__":
    main()
