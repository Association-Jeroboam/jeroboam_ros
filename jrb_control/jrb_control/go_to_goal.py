import traceback

from rcl_interfaces.msg import ParameterDescriptor, ParameterType, FloatingPointRange
import rclpy
from rclpy.time import Duration
import time
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rcl_interfaces.msg import SetParametersResult
from math import pi, radians
from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import Twist, PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Bool
from typing import Optional
import threading
from action_msgs.msg import GoalStatus
from rclpy.executors import MultiThreadedExecutor

from .goal_controller import GoalController, Pose2D
from jrb_msgs.action import GoToPose


class GoToGoalNode(Node):
    def __init__(self):
        super().__init__("go_to_goal")

        self.controller = GoalController()

        self.action_name = "diff_drive_go_to_goal"

        self.goal_handle_lock = threading.Lock()
        self.goal_handle: Optional[ServerGoalHandle] = None
        self.goal: Optional[Pose2D] = None
        self.action_server = ActionServer(
            self,
            GoToPose,
            self.action_name,
            self.on_execute,
            goal_callback=self.on_goal_callback,
            cancel_callback=self.on_cancel_callback,
            handle_accepted_callback=self.on_accepted_callback,
        )

        self.action_client = ActionClient(
            self,
            GoToPose,
            self.action_name,
        )

        self.dist_pub = self.create_publisher(Float32, "distance_to_goal", 10)
        self.twist_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.goal_achieved_pub = self.create_publisher(Bool, "goal_achieved", 1)

        self.get_logger().info(f"{self.get_name()} started")

        self.odom_sub = self.create_subscription(
            Odometry, "odometry", self.on_odometry, 10
        )
        self.goal_sub = self.create_subscription(
            PoseStamped,
            "goal_pose",
            self.on_goal,
            10,
        )

        self.declare_parameter(
            "rate",
            10.0,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                floating_point_range=[
                    FloatingPointRange(from_value=1.0, to_value=50.0, step=0.1)
                ],
            ),
        )
        self.declare_parameter(
            "kP",
            3.0,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                floating_point_range=[
                    FloatingPointRange(from_value=0.1, to_value=30.0, step=0.01)
                ],
            ),
        )

        self.declare_parameter(
            "kA",
            8.0,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                floating_point_range=[
                    FloatingPointRange(from_value=0.1, to_value=30.0, step=0.01)
                ],
            ),
        )

        self.declare_parameter(
            "kB",
            -1.5,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                floating_point_range=[
                    FloatingPointRange(from_value=-0.1, to_value=-30.0, step=0.01)
                ],
            ),
        )

        self.declare_parameter(
            "linear_tolerance",
            0.05,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                floating_point_range=[
                    FloatingPointRange(from_value=0.001, to_value=0.1, step=0.001)
                ],
            ),
        )

        self.declare_parameter(
            "angular_tolerance",
            3.0,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                floating_point_range=[
                    FloatingPointRange(from_value=0.1, to_value=10.0, step=0.1)
                ],
            ),
        )

        self.declare_parameter(
            "max_linear_speed",
            0.2,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                floating_point_range=[
                    FloatingPointRange(from_value=0.1, to_value=1.5, step=0.1)
                ],
            ),
        )

        self.declare_parameter(
            "min_linear_speed",
            0.0,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                floating_point_range=[
                    FloatingPointRange(from_value=0.0, to_value=0.1, step=0.001)
                ],
            ),
        )

        self.declare_parameter(
            "max_angular_speed",
            45.0,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                floating_point_range=[
                    FloatingPointRange(from_value=0.0, to_value=720.0, step=1.0)
                ],
            ),
        )

        self.declare_parameter(
            "min_angular_speed",
            0.0,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                floating_point_range=[
                    FloatingPointRange(from_value=0.0, to_value=10.0, step=0.01)
                ],
            ),
        )

        self.declare_parameter(
            "max_linear_acceleration",
            0.1,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                floating_point_range=[
                    FloatingPointRange(from_value=0.01, to_value=2.0, step=0.01)
                ],
            ),
        )

        self.declare_parameter(
            "max_angular_acceleration",
            20.0,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                floating_point_range=[
                    FloatingPointRange(from_value=0.01, to_value=1440.0, step=0.01)
                ],
            ),
        )

        self.declare_parameter(
            "forwardMovementOnly",
            False,
            descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_BOOL),
        )

        self.rate = self.get_parameter("rate").value
        self.dT = 1 / self.rate

        self.kP = self.get_parameter("kP").value
        self.kA = self.get_parameter("kA").value
        self.kB = self.get_parameter("kB").value
        self.controller.set_constants(self.kP, self.kA, self.kB)

        self.controller.set_linear_tolerance(
            self.get_parameter("linear_tolerance").value
        )
        self.controller.set_angular_tolerance(
            radians(self.get_parameter("angular_tolerance").value)
        )

        self.controller.set_max_linear_speed(
            self.get_parameter("max_linear_speed").value
        )
        self.controller.set_min_linear_speed(
            self.get_parameter("min_linear_speed").value
        )
        self.controller.set_max_angular_speed(
            radians(self.get_parameter("max_angular_speed").value)
        )
        self.controller.set_min_angular_speed(
            radians(self.get_parameter("min_angular_speed").value)
        )
        self.controller.set_max_linear_acceleration(
            self.get_parameter("max_linear_acceleration").value
        )
        self.controller.set_max_angular_acceleration(
            radians(self.get_parameter("max_angular_acceleration").value)
        )

        self.controller.set_forward_movement_only(
            self.get_parameter("forwardMovementOnly").value
        )

        self.add_on_set_parameters_callback(self.on_update_parameters)

        self.init_pose()

    def on_update_parameters(self, params):
        self.get_logger().info("Params updated")

        for param in params:
            if param.name == "rate":
                self.rate = param.value
                self.dT = 1 / self.rate

            elif param.name == "kP":
                self.kP = self.get_parameter("kp").value
                self.controller.set_constants(self.kP, self.kA, self.kB)
            elif param.name == "kA":
                self.kA = self.get_parameter("kA").value
                self.controller.set_constants(self.kP, self.kA, self.kB)
            elif param.name == "kB":
                self.kB = self.get_parameter("kB").value
                self.controller.set_constants(self.kP, self.kA, self.kB)
            elif param.name == "linear_tolerance":
                self.controller.set_linear_tolerance(param.value)
            elif param.name == "angular_tolerance":
                self.controller.set_angular_tolerance(radians(param.value))
            elif param.name == "max_linear_speed":
                self.controller.set_max_linear_speed(param.value)
            elif param.name == "min_linear_speed":
                self.controller.set_min_linear_speed(param.value)
            elif param.name == "max_angular_speed":
                self.controller.set_max_angular_speed(radians(param.value))
            elif param.name == "min_angular_speed":
                self.controller.set_min_angular_speed(radians(param.value))
            elif param.name == "max_linear_acceleration":
                self.controller.set_max_linear_acceleration(param.value)
            elif param.name == "max_angular_acceleration":
                self.controller.set_max_angular_acceleration(radians(param.value))
            elif param.name == "forwardMovementOnly":
                self.controller.set_forward_movement_only(param.value)

        return SetParametersResult(successful=True)

    def on_accepted_callback(self, goal_handle: ServerGoalHandle):
        # Single goal policy https://github.com/ros2/rclcpp/issues/759#issuecomment-539635000
        with self.goal_handle_lock:
            if self.goal_handle is not None and self.goal_handle.is_active:
                self.get_logger().warn(
                    "diff_drive_go_to_goal action received  before a previous action finished. Aborting previous action."
                )
                self.goal_handle.abort()

            self.goal_handle = goal_handle
        self.goal_handle.execute()

    def on_goal_callback(self, goal_request):
        # Accept all goals
        return GoalResponse.ACCEPT

    def on_cancel_callback(self, cancel_request):
        # Accept all cancel
        return CancelResponse.ACCEPT

    def on_execute(self, goal_handle: ServerGoalHandle):
        self.goal = self.get_angle_pose(goal_handle.request.pose.pose)
        self.get_logger().info(
            f"Goal: ({self.goal.x}, {self.goal.y}, ${self.goal.theta})"
        )

        success = True
        while rclpy.ok() and self.goal is not None:
            nextWork = self.get_clock().now() + Duration(seconds=self.dT)

            # Work
            self.publish()

            now = self.get_clock().now()
            if now > nextWork:
                ## TODO : use seconds_nanoseconds
                actual_dT = self.dT + (now - nextWork).nanoseconds / 1e9
                actual_rate = 1 / actual_dT
                self.get_logger().warn(
                    f"Rate failed: desired: {self.rate}, actual: {actual_rate}"
                )
            else:
                time_to_sleep = (nextWork - now).nanoseconds / 1e9
                time.sleep(time_to_sleep)

            if (
                goal_handle.status == GoalStatus.STATUS_ABORTED
                or not goal_handle.is_active
            ):
                self.get_logger().info("Goal aborted")
                self.goal_achieved_pub.publish(Bool(data=False))
                return GoToPose.Result(success=False)

            if goal_handle.is_cancel_requested:
                self.send_velocity(0.0, 0.0)
                success = False
                break

        if success:
            self.get_logger().info("Goal succeeded")
            goal_handle.succeed()
        else:
            self.get_logger().info("Goal cancelled")
            goal_handle.canceled()
            self.goal_achieved_pub.publish(Bool(data=False))

        self.goal_handle = None

        return GoToPose.Result(success=success)

    def init_pose(self):
        self.pose = Pose2D()
        self.pose.x = 0.0
        self.pose.y = 0.0
        self.pose.theta = 0.0

    def publish(self):
        if self.controller.at_goal(self.pose, self.goal):
            desired = Pose2D()
        else:
            desired = self.controller.get_velocity(self.pose, self.goal, self.dT)

        d = self.controller.get_goal_distance(self.pose, self.goal)
        self.dist_pub.publish(Float32(data=float(d)))

        self.send_velocity(desired.xVel, desired.thetaVel)

        # Forget the goal if achieved.
        if self.controller.at_goal(self.pose, self.goal):
            self.get_logger().info("Goal achieved")
            self.goal = None
            self.goal_achieved_pub.publish(Bool(data=True))

    def send_velocity(self, xVel, thetaVel):
        twist = Twist()
        twist.linear.x = xVel
        twist.angular.z = thetaVel
        self.twist_pub.publish(twist)

    def on_odometry(self, newPose):
        self.pose = self.get_angle_pose(newPose.pose.pose)

    def on_goal(self, goal):
        self.action_client.wait_for_server()
        action_goal = GoToPose.Goal()
        action_goal.pose.pose = goal.pose
        self.action_client.send_goal_async(action_goal)

    def get_angle_pose(self, quaternion_pose: Quaternion) -> Pose2D:
        q = [
            quaternion_pose.orientation.x,
            quaternion_pose.orientation.y,
            quaternion_pose.orientation.z,
            quaternion_pose.orientation.w,
        ]
        _, _, yaw = euler_from_quaternion(q)

        angle_pose = Pose2D()
        angle_pose.x = quaternion_pose.position.x
        angle_pose.y = quaternion_pose.position.y
        angle_pose.theta = yaw

        return angle_pose


def main(args=None):
    rclpy.init(args=args)

    node = GoToGoalNode()
    # Source: https://answers.ros.org/question/356434/ros-2-actionserver-callback-causes-subscriber-to-stop-receiving/
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
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
