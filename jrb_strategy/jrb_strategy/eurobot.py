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

from std_msgs.msg import Bool, String, Empty, Float32
from jrb_msgs.msg import ServoAngle
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped, PoseArray, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import math
from tf_transformations import quaternion_from_euler, euler_from_quaternion

from jrb_msgs.msg import (
    SampleDetectedArray,
    StackSample,
    PumpStatus,
    ValveStatus,
    ServoStatus,
    ServoConfig,
    ServoAngle,
)

from jrb_msgs.action import GoToPose


CAN_PROTOCOL_SERVO_ARM_LEFT_A = 0
CAN_PROTOCOL_SERVO_ARM_LEFT_B = 1
CAN_PROTOCOL_SERVO_ARM_LEFT_C = 2
CAN_PROTOCOL_SERVO_ARM_LEFT_D = 3
CAN_PROTOCOL_SERVO_ARM_LEFT_E = 4
CAN_PROTOCOL_SERVO_ARM_RIGHT_A = 5
CAN_PROTOCOL_SERVO_ARM_RIGHT_B = 6
CAN_PROTOCOL_SERVO_ARM_RIGHT_C = 7
CAN_PROTOCOL_SERVO_ARM_RIGHT_D = 8
CAN_PROTOCOL_SERVO_ARM_RIGHT_E = 9
CAN_PROTOCOL_SERVO_RAKE_LEFT_TOP = 10
CAN_PROTOCOL_SERVO_RAKE_LEFT_BOTTOM = 11
CAN_PROTOCOL_SERVO_RAKE_RIGHT_TOP = 12
CAN_PROTOCOL_SERVO_RAKE_RIGHT_BOTTOM = 13
CAN_PROTOCOL_SERVO_PUSH_ARM_LEFT = 14
CAN_PROTOCOL_SERVO_PUSH_ARM_RIGHT = 15
CAN_PROTOCOL_SERVO_MEASURE_FORK = 16
CAN_PROTOCOL_SERVO_PLIERS_INCLINATION = 17
CAN_PROTOCOL_SERVO_PLIERS = 18


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

        self.pub_twist = self.create_publisher(Twist, "/cmd_vel", 10)

        # self.pub_servo = self.create_publisher(ServoAngle, "servo_angle_target", 10)

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
        self.reset = Future()
        self.team = Future()
        self.strategy = None
        self.end_match_timer = None
        self.obstacle_stop = False
        self.currentX = None
        self.currentY = None
        self.currentTheta = None

        self.actuators = Actuators_robotbleu(self)

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

        self.pub_cmd_vel = self.create_publisher(Twist, "cmd_vel", 10)

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
        value = msg.data

        if value and not self.start.done():
            self.start.set_result("set")
            self.reset = Future()

        if not value and self.start.done():
            self.reset.set_result("set")

    def on_team(self, msg: String):
        if not self.start.done():
            self.team.set_result(msg.data)

    def on_strategy(self, msg: Bool):
        self.strategy = msg.data

    def on_obstacle_detected(self, msg: PoseArray):
        if len(msg.poses) > 0:
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
        if self.team.result() == "yellow":
            return (x, y, theta)
        else:
            return (x, 3.0 - y, 2 * pi - theta)

    def goto_cancel_callback(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info("GoTo goal successfully canceled")
        else:
            self.get_logger().info("GoTo goal failed to cancel")

        self.goto_goal_handle = None

    def goto_cancel(self):
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
        self.goto_goal_handle = None

    def toutdroit(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.2
        for i in range(20):
            self.pub_twist.publish(twist_msg)
            rclpy.spin_once(self)
            time.sleep(0.1)
        self.pub_twist.publish(Twist())
        rclpy.spin_once(self)

    def recalibration(self, theta, x=None, y=None):
        self.toutdroit()
        q = quaternion_from_euler(0, 0, radians(theta))

        pose_msg = PoseWithCovarianceStamped()

        pose_msg.header.stamp = self.get_clock().now().to_msg()

        pose_msg.pose.pose.position.x = float(x) if x is not None else self.currentX
        pose_msg.pose.pose.position.y = float(y) if y is not None else self.currentY

        pose_msg.pose.pose.orientation.x = q[0]
        pose_msg.pose.pose.orientation.y = q[1]
        pose_msg.pose.pose.orientation.z = q[2]
        pose_msg.pose.pose.orientation.w = q[3]

        self.pub_initialpose.publish(pose_msg)

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
        self.pub_initialpose.publish(
            initialpose_msg
        )  # 2e publish : work around transfert id reset (UAVCAN)
        # TODO: dirty hack, need a service / action to resolve when the pos is effectively set
        rclpy.spin_once(self)

        time.sleep(2)

    # def servo(self, name, activated):
    #     msg = ServoAngle()

    #     if name == "pince":
    #         msg.id = 18
    #         msg.radian = radians(50.0 if activated else 15)
    #     elif name == "inclinaison":
    #         msg.id = 17
    #         msg.radian = radians(165 if activated else 240)
    #     elif name == "fourche":
    #         msg.id = 16
    #         msg.radian = radians(155 if activated else 230)
    #     elif name == "bras droit":
    #         msg.id = 15
    #         msg.radian = radians(250 if activated else 155)
    #     elif name == "bras gauche":
    #         msg.id = 14
    #         msg.radian = radians(50.0 if activated else 145)

    #     self.pub_servo.publish(msg)

    def loop(self):
        while rclpy.ok():
            self.pub_twist.publish(Twist())
            self.get_logger().info("Init strategy. Wait for team...")
            rclpy.spin_until_future_complete(self, self.team)

            self.get_logger().info("Wait for start...")
            rclpy.spin_until_future_complete(self, self.start)

            self.get_logger().info(
                f"Start ! team: {self.team.result()} strategy: {str(self.strategy)}"
            )
            self.end_match_timer = self.create_timer(
                100, self.on_end_match, callback_group=self.cb_group
            )

            ######### Strategy here, written for YELLOW TEAM #########
            self.set_initialpose(0.865, 0.1, radians(90))

            if self.team.result() == "yellow":
                self.actuators.setPlierTilt("out")
                self.actuators.openPlier()

                self.goto(
                    1.5438363552093506,
                    0.39137980341911316,
                    radians(-41.358321086233005),
                )

                self.goto(
                    1.630508542060852, 0.3107624351978302, radians(-42.3681255748667)
                )

                self.toutdroit()
                # potentiellement : recalibration -45 deg orientation

                # Serre + eleve 45
                self.actuators.closePlier_stat()
                time.sleep(1)
                self.actuators.setPlierTiltAngle(205)

                self.goto(
                    0.28720876574516296,
                    0.22292561829090118,
                    radians(-175.57350942694433),
                )

                # baisse pince
                self.actuators.setPlierTiltAngle(170)
                time.sleep(1)

                self.recalibration(180, x=0.12)

                # lacher la statuette
                self.actuators.openPlier()
                time.sleep(1)

                self.goto(1.401249098777771, 0.61577181220054626, radians(-45))

                self.goto(
                    1.601249098777771, 0.41577181220054626, radians(-43.95235404529698)
                )

                # Attraper replique
                self.actuators.openPlier()
                time.sleep(1)
                self.actuators.setPlierTilt("in")
                time.sleep(1)
                self.actuators.closePlier_rep()

                # self.goto(
                #     1.6761348247528076, 0.33661699295043945, radians(-45.39803376505072)
                # )
                self.toutdroit()

                # poser replique
                self.actuators.setPlierTilt("out")
                time.sleep(1)
                self.actuators.openPlier()

                self.goto(
                    1.6932655572891235, 0.8784884023666382, radians(0.7448462132756761)
                )

                self.actuators.closePlier_full()

                self.recalibration(0, 1.9)

                self.goto(
                    0.6886870265007019, 1.5326829195022583, radians(-77.4526634313721)
                )

                self.goto(
                    0.731564998626709, 1.1083611249923706, radians(-81.97548038594752)
                )

                self.goto(
                    0.6989494562149048, 0.2988869249820709, radians(-95.18329640489486)
                )
            elif self.team.result() == "purple":
                self.actuators.setPlierTilt("out")
                self.actuators.closePlier_full()

                self.goto(1.16, 0.9441, radians(104.6))

                self.goto(0.69, 1.378, radians(-90))

                self.goto(0.69, 0.3077, radians(-90))

                self.goto(1.49, 0.51, radians(90))

                # Pousse des carr??s de fouille
                if self.team.result() == "yellow":
                    self.goto(1.59, 0.832, radians(0))
                    self.goto(1.90, 0.77, radians(0))
                else:
                    self.goto(1.59, 0.832 + 0.1, radians(0))
                    self.goto(1.90, 0.77 + 0.1, radians(0))

                self.goto(0.865, 0.25, radians(90))

            ######### End strategy ##########

            self.get_logger().info("Strategy finished !")

            rclpy.spin_until_future_complete(self, self.end_match)

            self.get_logger().info("End match !")
            self.pub_end_match.publish(Empty())

            rclpy.spin_until_future_complete(self, self.reset)

            self.start = Future()
            self.end_match = Future()


class Actuators_robotbleu(Node):
    def __init__(self, node):
        super().__init__("actuators")
        self.get_logger().info("init actuator_robotbleu")

        self.node = node

        self.xl320_status = {}
        self.present_resistance = 0

        self.sub_xl320_status = self.create_subscription(
            ServoStatus, "servo_status", self.xl320_status_cb, 10
        )
        self.sub_xl320_status = self.create_subscription(
            Float32, "resistance_status", self.present_resistance_cb, 10
        )
        self.pub_xl320_target = self.create_publisher(
            ServoAngle, "servo_angle_target", 10
        )
        self.pub_xl320_config = self.create_publisher(ServoConfig, "servo_config", 10)

        # config msg
        self.arm_left_config_msg = ServoConfig()
        self.arm_right_config_msg = ServoConfig()
        self.ohm_reader_config_msg = ServoConfig()
        self.plier_tilt_config_msg = ServoConfig()
        self.plier_config_msg = ServoConfig()

        self.init_actuators()

    def xl320_status_cb(self, msg: ServoStatus):
        if not msg.id in self.xl320_status:
            self.xl320_status[msg.id] = ServoStatus()
        self.xl320_status[msg.id] = msg

    def present_resistance_cb(self, msg: Float32):
        self.present_resistance = msg.data

    def init_actuators(self):

        # config servo
        self.arm_left_config_msg.id = CAN_PROTOCOL_SERVO_PUSH_ARM_LEFT
        self.arm_right_config_msg.id = CAN_PROTOCOL_SERVO_PUSH_ARM_RIGHT
        self.ohm_reader_config_msg.id = CAN_PROTOCOL_SERVO_MEASURE_FORK
        self.plier_tilt_config_msg.id = CAN_PROTOCOL_SERVO_PLIERS_INCLINATION
        self.plier_config_msg.id = CAN_PROTOCOL_SERVO_PLIERS

        self.arm_left_config_msg.torque_limit = (
            self.arm_right_config_msg.torque_limit
        ) = (
            self.ohm_reader_config_msg.torque_limit
        ) = self.plier_tilt_config_msg.torque_limit = 1023

        self.arm_left_config_msg.moving_speed = 512  # max: 1024
        self.arm_right_config_msg.moving_speed = 512  # max: 1024
        self.ohm_reader_config_msg.moving_speed = 300  # max: 1024
        self.plier_tilt_config_msg.moving_speed = 200  # max: 1024

        self.arm_left_config_msg.pid.pid = (
            self.arm_right_config_msg.pid.pid
        ) = self.ohm_reader_config_msg.pid.pid = [
            32.0,
            0.0,
            0.0,
        ]

        self.plier_tilt_config_msg.pid.pid = [80.0, 10.0, 0.0]

        self.pub_xl320_config.publish(self.arm_left_config_msg)
        self.pub_xl320_config.publish(self.arm_right_config_msg)
        self.pub_xl320_config.publish(self.ohm_reader_config_msg)
        self.pub_xl320_config.publish(self.plier_tilt_config_msg)
        self.pub_xl320_config.publish(self.plier_config_msg)

        self.get_logger().info("configs published")
        time.sleep(0.5)

        # test servo
        self.setOhmReader(230)
        self.setPlierTilt("in")
        self.setArm("left", "in")
        self.setArm("right", "in")
        self.openPlier()

        self.get_logger().info("init OK")

    def setArm(self, side, state):
        angle_msg = ServoAngle()

        if side == "left":
            angle_msg.id = self.arm_left_config_msg.id
            if state == "in":
                angle_msg.radian = math.radians(150)
            elif state == "out ":
                angle_msg.radian = math.radians(50)

        elif side == "right":
            angle_msg.id = self.arm_right_config_msg.id
            if state == "in":
                angle_msg.radian = math.radians(150)
            elif state == "out ":
                angle_msg.radian = math.radians(250)

        else:
            return

        self.pub_xl320_target.publish(angle_msg)

    def setOhmReader(self, degrees):
        angle_msg = ServoAngle()
        angle_msg.id = self.ohm_reader_config_msg.id
        angle_msg.radian = math.radians(degrees)
        self.pub_xl320_target.publish(angle_msg)

    def setPlierTilt(self, state):
        if state == "in":
            self.setPlierTiltAngle(240)
        elif state == "out":
            self.setPlierTiltAngle(155)

    def setPlierTiltAngle(self, degrees):
        angle_msg = ServoAngle()
        angle_msg.id = self.plier_tilt_config_msg.id
        angle_msg.radian = math.radians(degrees)
        self.pub_xl320_target.publish(angle_msg)

    def openPlier(self):
        angle_msg = ServoAngle()
        angle_msg.id = self.plier_config_msg.id
        angle_msg.radian = math.radians(15)
        self.pub_xl320_target.publish(angle_msg)

    def closePlier_stat(self):
        print("in closePlierStat")
        angle_msg = ServoAngle()
        angle_msg.id = self.plier_config_msg.id
        angle_msg.radian = math.radians(51)
        self.pub_xl320_target.publish(angle_msg)

    def closePlier_rep(self):
        print("in closePlierRep")
        angle_msg = ServoAngle()
        angle_msg.id = self.plier_config_msg.id
        angle_msg.radian = math.radians(55)
        self.pub_xl320_target.publish(angle_msg)

    def closePlier_full(self):
        print("in closePlierRep")
        angle_msg = ServoAngle()
        angle_msg.id = self.plier_config_msg.id
        angle_msg.radian = math.radians(85)
        self.pub_xl320_target.publish(angle_msg)


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
        node.get_logger().info("Error while stopping the node")
        node.get_logger().info(traceback.format_exc())
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()

        if thread is not None:
            thread.join()


if __name__ == "__main__":
    main()
