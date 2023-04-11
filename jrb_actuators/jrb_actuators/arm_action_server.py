#!/usr/bin/env python3

import sys
import time
import threading
import traceback

import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from action_msgs.msg import GoalStatus
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from jrb_msgs.action import GoToPose
from std_msgs.msg import String


class ArmActionServer(Node):
    def __init__(self, arm_name):
        super().__init__(arm_name + "_arm_action_server")

        self.arm_name = arm_name
        self.action_name = arm_name + "_arm_go_to_goal"

        # Subscribers
        self.subscription = self.create_subscription(
            String, "/test", self.test_callback, 10
        )

        # Internal data state
        self.goal_handle_lock = threading.Lock()
        self.execute_lock = threading.Lock()
        self.current_goal_handle = None

        self._action_server = ActionServer(
            self,
            GoToPose,
            self.action_name,
            self.execute_callback,
            goal_callback=self.on_goal_callback,
            handle_accepted_callback=self.on_accepted_callback,
            cancel_callback=self.on_cancel_callback,
        )

        self.get_logger().info(f"{self.action_name} action server initialized")

    def is_goal_handle_stopping(self, goal_handle):
        return (
            goal_handle.is_cancel_requested
            or goal_handle.status == GoalStatus.STATUS_ABORTED
            or not goal_handle.is_active
        )

    def wait_for_seconds(self, seconds, goal_handle, step=0.1):
        if seconds <= step:
            time.sleep(seconds)
            return

        start_time = time.time()
        end_time = start_time + seconds

        while (
            (remaining_time := end_time - time.time()) > 0
            and rclpy.ok()
            and not self.is_goal_handle_stopping(goal_handle)
        ):
            time.sleep(min(remaining_time, step))

    def on_goal_callback(self, _):
        # Accept all goals
        return GoalResponse.ACCEPT

    def on_cancel_callback(self, _):
        # Accept all cancel
        return CancelResponse.ACCEPT

    def on_accepted_callback(self, goal_handle):
        try:
            with self.goal_handle_lock:
                if (
                    self.current_goal_handle is not None
                    and self.current_goal_handle.is_active
                ):
                    self.get_logger().warn(
                        "New goal received  before a previous finished. Aborting previous goal."
                    )
                    self.current_goal_handle.abort()

                self.current_goal_handle = goal_handle
                self.current_goal_handle.execute()
        except Exception:
            self.get_logger().error(traceback.format_exc())

    def execute_callback(self, goal_handle):
        try:
            with self.execute_lock:
                self.get_logger().info("Executing goal...")

                self.setup()

                # Work loop
                while True:
                    # Handle node shutting down
                    if not rclpy.ok():
                        print("Node is shutting down, early exit execute_callback")
                        return GoToPose.Result(success=False)

                    # Handle goal cancelled
                    if goal_handle.is_cancel_requested:
                        self.get_logger().info("Goal cancelled")
                        goal_handle.canceled()
                        return GoToPose.Result(success=False)

                    # Handle goal aborted
                    if (
                        goal_handle.status == GoalStatus.STATUS_ABORTED
                        or not goal_handle.is_active
                    ):
                        self.get_logger().info("Goal aborted")
                        return GoToPose.Result(success=False)

                    # Loop and end the work loop if there is a result
                    if result := self.loop(goal_handle) is not None:
                        return result

        except Exception:
            self.get_logger().error(traceback.format_exc())
            goal_handle.abort()
            return GoToPose.Result(success=False)

    def test_callback(self, msg):
        self.get_logger().info("test: " + msg.data)
        self.test = msg.data

    def setup(self):
        self.i = 1
        self.partial_sequence = [0, 1]
        self.test = None

    def loop(self, goal_handle):
        # Success condition
        if self.i >= 10:
            goal_handle.succeed()
            return GoToPose.Result(success=True)

        # Work
        self.wait_for_seconds(2.0, goal_handle)
        self.partial_sequence.append(
            self.partial_sequence[self.i] + self.partial_sequence[self.i - 1]
        )

        print("test: ", self.test)

        # Publish feedback
        self.get_logger().info("Feedback: {0}".format(self.partial_sequence))
        goal_handle.publish_feedback(GoToPose.Feedback(processing=True))

        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print("Usage: python3 arm_action_server.py <arm_name>")
        return

    arm_name = sys.argv[1]
    arm_action_server = ArmActionServer(arm_name)

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(arm_action_server)

    try:
        executor.spin()
    except KeyboardInterrupt:
        print("Stopping the node cleanly...")
    except Exception:
        print("Error while stopping the node")
        print(arm_action_server.format_exc())
        raise
    finally:
        arm_action_server.destroy_node()


if __name__ == "__main__":
    main()
