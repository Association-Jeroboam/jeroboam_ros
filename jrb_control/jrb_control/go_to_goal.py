import traceback
from math import pi, radians
from typing import Optional
import threading

from rcl_interfaces.msg import ParameterDescriptor, ParameterType, FloatingPointRange
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSReliabilityPolicy, QoSProfile
import rclpy
import rclpy.time
import time
from rcl_interfaces.msg import SetParametersResult
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration

from tf_transformations import euler_from_quaternion
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs
from geometry_msgs.msg import Twist, PoseStamped, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Bool
from action_msgs.msg import GoalStatus
from jrb_msgs.action import GoToPose

from .goal_controller import GoalController, Pose2D


# TODO : spin in place
# TODO : timeout
# TODO : oscillations rejection
# TODO : max vel lin / ang
# TODO : deceleration ramp
# TODO : frequency
# TODO : feedback with remaining distance, angle, and navigation_duration
# TODO : soft ESTOP
class GoToGoalNode(Node):
    def __init__(self):
        super().__init__("go_to_goal")
        self.get_logger().info(f"{self.get_name()} started")

        cmd_vel_qos_profile = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy(QoSReliabilityPolicy.BEST_EFFORT),
        )

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

        self.dist_pub = self.create_publisher(Float32, "/distance_to_goal", 10)
        self.twist_pub = self.create_publisher(Twist, "/cmd_vel_nav", 1)
        self.goal_achieved_pub = self.create_publisher(Bool, "/goal_achieved", 1)
        self.pub_debug_current_goal = self.create_publisher(
            Marker, "/debug/current_goal", 1
        )

        self.odom_sub = self.create_subscription(
            Odometry, "/odometry", self.on_odometry, 10
        )
        self.goal_sub = self.create_subscription(
            PoseStamped,
            "/goal_pose",
            self.on_goal,
            10,
        )
        self.stucked_spub = self.create_subscription(
            Bool, "/stuck", self.on_stucked, 10
        )

        # Tf subscriber
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.declare_parameter(
            "rate",
            50.0,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                floating_point_range=[
                    FloatingPointRange(from_value=1.0, to_value=50.0, step=0.1)
                ],
            ),
        )
        self.declare_parameter(
            "kP",
            1.5,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                floating_point_range=[
                    FloatingPointRange(from_value=0.1, to_value=30.0, step=0.01)
                ],
            ),
        )

        self.declare_parameter(
            "kA",
            9.0,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                floating_point_range=[
                    FloatingPointRange(from_value=0.1, to_value=30.0, step=0.01)
                ],
            ),
        )

        self.declare_parameter(
            "kB",
            -3.0,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                floating_point_range=[
                    FloatingPointRange(from_value=-30.0, to_value=-0.1, step=0.01)
                ],
            ),
        )

        self.declare_parameter(
            "linear_tolerance",
            0.01,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                floating_point_range=[
                    FloatingPointRange(from_value=0.001, to_value=0.1, step=0.001)
                ],
            ),
        )

        self.declare_parameter(
            "angular_tolerance",
            1.5,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                floating_point_range=[
                    FloatingPointRange(from_value=0.1, to_value=10.0, step=0.1)
                ],
            ),
        )

        self.declare_parameter(
            "max_linear_speed",
            0.5,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                floating_point_range=[
                    FloatingPointRange(from_value=0.1, to_value=1.5, step=0.1)
                ],
            ),
        )

        self.declare_parameter(
            "max_angular_speed",
            360.0,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                floating_point_range=[
                    FloatingPointRange(from_value=1.0, to_value=720.0, step=1.0)
                ],
            ),
        )

        self.declare_parameter(
            "max_linear_acceleration",
            0.5,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                floating_point_range=[
                    FloatingPointRange(from_value=0.1, to_value=5.0, step=0.1)
                ],
            ),
        )

        self.declare_parameter(
            "max_angular_acceleration",
            90.0,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                floating_point_range=[
                    FloatingPointRange(from_value=1.0, to_value=1440.0, step=1.0)
                ],
            ),
        )

        self.declare_parameter(
            "max_linear_jerk",
            0.5,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                floating_point_range=[
                    FloatingPointRange(from_value=0.1, to_value=10.0, step=0.1)
                ],
            ),
        )

        self.declare_parameter(
            "max_angular_jerk",
            45.0,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                floating_point_range=[
                    FloatingPointRange(from_value=1.0, to_value=2880.0, step=1.0)
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

        self.controller.set_max_angular_speed(
            radians(self.get_parameter("max_angular_speed").value)
        )

        self.controller.set_max_linear_acceleration(
            self.get_parameter("max_linear_acceleration").value
        )

        self.controller.set_max_angular_acceleration(
            radians(self.get_parameter("max_angular_acceleration").value)
        )

        self.controller.set_max_linear_jerk(self.get_parameter("max_linear_jerk").value)

        self.controller.set_max_angular_jerk(
            radians(self.get_parameter("max_angular_jerk").value)
        )

        self.controller.set_forward_movement_only(
            self.get_parameter("forwardMovementOnly").value
        )

        self.add_on_set_parameters_callback(self.on_update_parameters)

        self.init_pose()

    def publish_marker(self, stamp, pose):
        self.current_goal_marker_msg = Marker()

        self.current_goal_marker_msg.header.frame_id = "odom"
        self.current_goal_marker_msg.header.stamp = stamp

        self.current_goal_marker_msg.lifetime = Duration()

        self.current_goal_marker_msg.type = Marker.ARROW
        self.current_goal_marker_msg.action = Marker.ADD
        self.current_goal_marker_msg.id = 0

        self.current_goal_marker_msg.pose = pose
        self.current_goal_marker_msg.scale.x = 0.3
        self.current_goal_marker_msg.scale.y = 0.05
        self.current_goal_marker_msg.scale.z = 0.05

        self.current_goal_marker_msg.color.a = 1.0
        self.current_goal_marker_msg.color.r = 0.0
        self.current_goal_marker_msg.color.g = 0.0
        self.current_goal_marker_msg.color.b = 1.0

        self.pub_debug_current_goal.publish(self.current_goal_marker_msg)

    def on_update_parameters(self, params):
        for param in params:
            if param.name == "rate":
                self.rate = param.value
                self.dT = 1 / self.rate
            elif param.name == "kP":
                self.kP = param.value
                self.controller.set_constants(self.kP, self.kA, self.kB)
            elif param.name == "kA":
                self.kA = param.value
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
            elif param.name == "max_angular_speed":
                self.controller.set_max_angular_speed(radians(param.value))
            elif param.name == "max_linear_acceleration":
                self.controller.set_max_linear_acceleration(param.value)
            elif param.name == "max_angular_acceleration":
                self.controller.set_max_angular_acceleration(radians(param.value))
            elif param.name == "max_linear_jerk":
                self.controller.set_max_linear_jerk(param.value)
            elif param.name == "max_angular_jerk":
                self.controller.set_max_angular_jerk(radians(param.value))
            elif param.name == "forwardMovementOnly":
                self.controller.set_forward_movement_only(param.value)

        self.get_logger().info("Params updated")

        return SetParametersResult(successful=True)

    def on_stucked(self, stuckedMsg: Bool):
        if stuckedMsg.data:
            with self.goal_handle_lock:
                if self.goal_handle is not None and self.goal_handle.is_active:
                    self.get_logger().warn("Robot stucked, aborting action")
                    self.goal_handle.abort()

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

    def convert_pose_to_map(self, pose_stamped: PoseStamped):
        if pose_stamped.header.frame_id == "map":
            return pose_stamped
        else:
            try:
                # Wait for the transform to become available
                self.tf_buffer.can_transform(
                    "map",
                    pose_stamped.header.frame_id,
                    pose_stamped.header.stamp,
                    timeout=rclpy.time.Duration(seconds=0.5),
                )

                # Transform the pose
                transform = self.tf_buffer.lookup_transform(
                    "map", pose_stamped.header.frame_id, pose_stamped.header.stamp
                )
                pose_transformed = tf2_geometry_msgs.do_transform_pose_stamped(
                    pose_stamped, transform
                )

                return pose_transformed
            except Exception as e:
                self.get_logger().error("Failed to convert pose: %s" % e)
                return None

    def on_execute(self, goal_handle: ServerGoalHandle):
        # Goal can be a pose in any frame (in the robot's frame to do relative moves)
        # Convert the goal to the map frame
        goal_pose = goal_handle.request.pose
        self.rotation = goal_handle.request.rotation

        if self.rotation:
            goal_pose.pose.position.x = self.pose.x
            goal_pose.pose.position.y = self.pose.y

        goal_pose_map = self.convert_pose_to_map(goal_pose)

        if goal_pose_map is None:
            self.stop_robot()
            self.goal_achieved_pub.publish(Bool(data=False))
            return GoToPose.Result(success=False)

        # Convert the Pose ROS msg to the Pose2D internall class
        self.goal = self.get_angle_pose(goal_pose_map.pose)

        if self.rotation:
            self.get_logger().info(f"ROTATION Goal: ${self.goal.theta})")
        else:
            self.get_logger().info(
                f"Goal: ({self.goal.x}, {self.goal.y}, ${self.goal.theta})"
            )

        # Debug marker
        self.publish_marker(self.get_clock().now().to_msg(), goal_pose.pose)

        success = True
        while rclpy.ok() and self.goal is not None:
            nextWork = self.get_clock().now() + rclpy.time.Duration(seconds=self.dT)

            # Work
            self.work()

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
                self.stop_robot()
                self.goal_achieved_pub.publish(Bool(data=False))
                return GoToPose.Result(success=False)

            if goal_handle.is_cancel_requested:
                self.stop_robot()
                success = False
                break

        if success:
            self.get_logger().info("Goal succeeded")
            goal_handle.succeed()
            self.goal_achieved_pub.publish(Bool(data=True))
        else:
            self.get_logger().warn("Goal cancelled")
            goal_handle.canceled()
            self.goal_achieved_pub.publish(Bool(data=False))

        self.stop_robot()
        self.goal_handle = None

        return GoToPose.Result(success=success)

    def stop_robot(self):
        self.send_velocity(0.0, 0.0)
        self.controller.last_linear_speed_cmd = [0.0, 0.0]
        self.controller.last_angular_speed_cmd = [0.0, 0.0]

    def init_pose(self):
        self.pose = Pose2D()
        self.pose.x = 0.0
        self.pose.y = 0.0
        self.pose.theta = 0.0

    def work(self):
        if self.controller.at_goal(self.pose, self.goal, self.rotation):
            desired = Pose2D()
        else:
            desired = self.controller.get_velocity(
                self.pose, self.goal, self.dT, self.rotation
            )

        d = self.controller.get_goal_distance(self.pose, self.goal)
        self.dist_pub.publish(Float32(data=float(d)))

        if self.rotation:
            self.send_velocity(0.0, desired.thetaVel)
        else:
            self.send_velocity(desired.xVel, desired.thetaVel)

        # Forget the goal if achieved.
        if self.controller.at_goal(self.pose, self.goal, self.rotation):
            self.get_logger().info("Goal achieved")
            self.controller.last_linear_speed_cmd = [0.0, 0.0]
            self.controller.last_angular_speed_cmd = [0.0, 0.0]
            self.goal = None
            self.goal_achieved_pub.publish(Bool(data=True))

    def send_velocity(self, xVel, thetaVel):
        twist = Twist()
        twist.linear.x = float(xVel)
        twist.angular.z = float(thetaVel)
        self.twist_pub.publish(twist)

    def on_odometry(self, newPose: Odometry):
        self.pose = self.get_angle_pose(newPose.pose.pose)

    def on_stucked(self, stuckedMsg: Bool):
        if stuckedMsg.data:
            with self.goal_handle_lock:
                if self.goal_handle is not None and self.goal_handle.is_active:
                    self.get_logger().warn("Robot stucked, aborting action")
                    self.goal_handle.abort()

    def on_goal(self, goal):
        self.action_client.wait_for_server()
        action_goal = GoToPose.Goal()
        action_goal.pose: PoseStamped = goal
        self.action_client.send_goal_async(action_goal)

    def get_angle_pose(self, pose: Pose) -> Pose2D:
        q = [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]
        _, _, yaw = euler_from_quaternion(q)

        angle_pose = Pose2D()
        angle_pose.x = pose.position.x
        angle_pose.y = pose.position.y
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
