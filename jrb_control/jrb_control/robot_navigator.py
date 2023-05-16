#! /usr/bin/env python3

from enum import Enum
from math import pi, atan2, radians, degrees
import traceback
import threading
import time
from typing import Optional

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import (
    Quaternion,
    PoseWithCovarianceStamped,
    PoseStamped,
    Pose,
    Pose2D,
    Twist,
)
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String, Empty, Float32, Float64
from jrb_msgs.action import GoToPose

import rclpy
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.duration import Duration as rclpyDuration
from rclpy.node import Node
from rclpy.task import Future
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from tf_transformations import quaternion_from_euler, euler_from_quaternion


def interact():
    import code

    code.InteractiveConsole(locals=globals()).interact()


def create_quaternion_msg_from_yaw(yaw):
    quat = quaternion_from_euler(0, 0, yaw)
    return Quaternion(**dict(zip(["x", "y", "z", "w"], quat)))


def yaw_from_quaternion_msg(q: Quaternion):
    _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

    return yaw


class NavTaskResult(Enum):
    UNKNOWN = 0
    SUCCEEDED = 1
    CANCELED = 2
    FAILED = 3


class BasicNavigator(Node):
    def __init__(self, node_name="basic_navigator"):
        super().__init__(node_name=node_name)
        self.initial_pose = PoseStamped()
        self.initial_pose.header.frame_id = "map"
        self.pose: Optional[Pose2D] = None

        self.nav_goal_handle: Optional[ClientGoalHandle] = None
        self.nav_result_future: Optional[GoToPose.Result] = None
        self.nav_feedback: Optional[GoToPose.Feedback] = None
        self.nav_status: NavTaskResult = NavTaskResult.UNKNOWN

        self.latched_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.go_to_pose = ActionClient(self, GoToPose, "diff_drive_go_to_goal")

        self.localization_pose_sub = self.create_subscription(
            Odometry,
            "/odometry",
            self._odometryCallback,
            1,
        )
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "initialpose", 10
        )
        self.pub_twist = self.create_publisher(Twist, "/cmd_vel", 10)

        self.debug("Waiting for 'GoToPose' action server")
        while not self.go_to_pose.wait_for_server(timeout_sec=5.0):
            self.info("'GoToPose' action server not available, waiting...")

        self.sleep(1)

    def destroyNode(self):
        self.destroy_node()

    def destroy_node(self):
        self.go_to_pose.destroy()
        super().destroy_node()

    def setInitialPose(self, initial_pose: Pose):
        """Set the initial pose to the localization system."""
        self.initial_pose = initial_pose

        msg = PoseWithCovarianceStamped()
        msg.pose.pose = self.initial_pose.pose
        msg.header.frame_id = self.initial_pose.header.frame_id
        msg.header.stamp = self.initial_pose.header.stamp

        self.info("Publishing Initial Pose")
        self.initial_pose_pub.publish(msg)

    def goToPose(self, pose: Pose2D, relative=False, blocking=True):
        """Send a `GoToPose` action request."""
        self.debug("Waiting for 'GoToPose' action server")
        while not self.go_to_pose.wait_for_server(timeout_sec=5.0):
            self.info("'GoToPose' action server not available, waiting...")

        frame_id = "map" if not relative else "base_footprint"
        goal_msg = GoToPose.Goal()
        goal_msg.pose = self._createPoseMsgFromPose2D(pose, frame_id=frame_id)

        self.info(
            "Navigating to goal: "
            + str(pose.x)
            + " "
            + str(pose.y)
            + " "
            + str(pose.theta)
            + "..."
        )

        send_goal_future = self.go_to_pose.send_goal_async(goal_msg)

        rclpy.spin_until_future_complete(self, send_goal_future)

        self.nav_goal_handle = send_goal_future.result()

        if not self.nav_goal_handle.accepted:
            self.error(
                "Goal to "
                + str(pose.pose.position.x)
                + " "
                + str(pose.pose.position.y)
                + " "
                + str(pose.theta)
                + " was rejected!"
            )
            return False

        self.nav_result_future = self.nav_goal_handle.get_result_async()

        if blocking:
            self.waitForNavTaskToComplete()

    def spin(self, yaw: float, relative=False):
        pose = Pose2D()
        pose.theta = float(yaw)

        return self.goToPose(pose, relative)

    def forward(self, dist=0.15, backup_speed=0.025):
        pose = Pose2D()
        pose.x = dist

        return self.goToPose(pose, relative=True)

    def backup(self, dist=0.15, backup_speed=0.025):
        pose = Pose2D()
        pose.x = -dist

        return self.goToPose(pose, relative=True)

    def cancelNavTask(self):
        """Cancel pending task request of any type."""
        self.info("Canceling current task.")

        if self.nav_result_future:
            future = self.nav_goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)

        return

    def isNavTaskComplete(self):
        """Check if the task request of any type is complete yet."""
        if not self.nav_result_future:
            # task was cancelled or completed
            return True

        rclpy.spin_until_future_complete(self, self.nav_result_future, timeout_sec=0.10)

        if self.nav_result_future.result():
            self.nav_status = self.nav_result_future.result().status

            if self.nav_status != GoalStatus.STATUS_SUCCEEDED:
                self.debug(f"Task with failed with status code: {self.nav_status}")

                return True
        else:
            # Timed out, still processing, not complete yet
            return False

        self.debug("Task succeeded!")

        return True

    def waitForNavTaskToComplete(self):
        while not self.isNavTaskComplete():
            pass

        result = self.getResult()
        if result == NavTaskResult.SUCCEEDED:
            print("Goal succeeded!")
            return True
        elif result == NavTaskResult.CANCELED:
            print("Goal was canceled!")
            return False
        elif result == NavTaskResult.FAILED:
            print("Goal failed!")
            return False

    def sleep(self, seconds: float):
        rclpy.spin_until_future_complete(self, Future(), timeout_sec=seconds)

    def getFeedback(self):
        """Get the pending action feedback message."""
        return self.nav_feedback

    def getResult(self):
        """Get the pending action result message."""
        if self.nav_status == GoalStatus.STATUS_SUCCEEDED:
            return NavTaskResult.SUCCEEDED
        elif self.nav_status == GoalStatus.STATUS_ABORTED:
            return NavTaskResult.FAILED
        elif self.nav_status == GoalStatus.STATUS_CANCELED:
            return NavTaskResult.CANCELED
        else:
            return NavTaskResult.UNKNOWN

    def stop(self):
        self.info("Stopping robot")
        self.cancelNavTask()
        self.pub_twist.publish(Twist())

    def _odometryCallback(self, msg: Odometry):
        yaw = yaw_from_quaternion_msg(msg.pose.pose.orientation)
        self.pose = Pose2D()
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y
        self.pose.theta = degrees(yaw)
        return

    def _feedbackCallback(self, msg: GoToPose.Feedback):
        self.debug("Received action feedback message")
        self.nav_feedback = msg
        return

    def _createPoseMsgFromPose2D(self, pose: Pose2D, frame_id="map") -> PoseStamped:
        pose_stamped_msg = PoseStamped()
        pose_stamped_msg.header.stamp = self.get_clock().now().to_msg()
        pose_stamped_msg.header.frame_id = frame_id
        pose_stamped_msg.pose.position.x = pose.x
        pose_stamped_msg.pose.position.y = pose.y
        pose_stamped_msg.pose.orientation = create_quaternion_msg_from_yaw(
            radians(pose.theta)
        )

        return pose_stamped_msg

    def printPose(self):
        self.sleep(0.2)

        if self.pose is None:
            return

        self.info(
            f"x: {self.pose.x:.3f} y: {self.pose.y:.3f} yaw: {self.pose.theta:.1f}"
        )

    def info(self, msg):
        self.get_logger().info(msg)
        return

    def warn(self, msg):
        self.get_logger().warn(msg)
        return

    def error(self, msg):
        self.get_logger().error(msg)
        return

    def debug(self, msg):
        self.get_logger().debug(msg)
        return


def main():
    rclpy.init()

    nav = BasicNavigator()
    from ptpython.repl import embed

    embed(globals(), locals())


if __name__ == "__main__":
    main()
