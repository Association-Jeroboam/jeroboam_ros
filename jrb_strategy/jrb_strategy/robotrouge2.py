import traceback
import threading
import time
import sys
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
from std_msgs.msg import Bool, Int8, Int16, UInt16, UInt8, String, Empty
from geometry_msgs.msg import PoseStamped, PoseArray, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import math
from tf_transformations import quaternion_from_euler, euler_from_quaternion

from jrb_msgs.msg import (
    SampleDetectedArray,
    StackSample,
    ServoStatus,
    ServoConfig,
    ServoAngle,
)

from jrb_msgs.action import GoToPose

MATCH_DURATION = 100 #s
EXPECTED_STATIC_SCORE = 0 

class EurobotStrategyNode(Node):
    def __init__(self):
        super().__init__("robotrouge_eurobot_strategy")
        self.get_logger().info("init")

        latchedQoS = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=1,
        )

        self.cb_group = ReentrantCallbackGroup()


        self.pub_end_match = self.create_publisher(Empty, "strategy/end_match", 1)
        self.pub_initialpose = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self.pub_twist = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_score = self.create_publisher(String, "screen/score", 10)

        self.goto_action_client = ActionClient(
            self, GoToPose, "diff_drive_go_to_goal", callback_group=self.cb_group
        )

        self.get_logger().info("Wait for GoTo action server...")
        self.goto_action_client.wait_for_server()

        self.goto_goal_msg = GoToPose.Goal()
        self.goto_goal_msg.pose.header.frame_id = "map" 
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

        self.sub_emergency = self.create_subscription(
            Bool, "/hardware/emergency/status", self.on_emergency, latchedQoS
        )
        
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
            1,
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

    def on_panier(self, msg: UInt8):
        value = msg.data
        self.panier = value
        self.printScore()
        

    def printScore(self):
        score = EXPECTED_STATIC_SCORE
        self.pub_score.publish(String(data=str(score)))

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
        if not self.start.done():
            return

        if self.obstacle_stop:
            return

        if len(msg.poses) > 0:
            self.get_logger().warn("Obstacle detected ! Cancelling all goals")
            self.obstacle_stop = True
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

    def on_emergency(self, msg: Bool):
        if msg.data and self.start.done():
            self.get_logger().warn("ESTOP on")
            self.on_end_match()

    def on_end_match(self, e):
        if self.end_match.done():
            return

        if e is None:
            self.get_logger().info("[MATCH] End match because of timer")
        else:
            self.get_logger().info("[MATCH] End match ")

        self.end_match.set_result("set")

        if self.end_match_timer is not None:
            self.end_match_timer.cancel()

        self.goto_cancel()
        self.pub_twist.publish(Twist())
        time.sleep(2)
        sys.exit(1)

    def get_pose(self, x, y, theta):
        # yellow = green = droite
        # purple = blue = gauche

        if self.team.result() == "purple":
            return (x, y, theta)
        else:
            return (2 - x, y, pi - theta)

    def goto_cancel_callback(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info("[GOTO] goal successfully canceled")
        else:
            self.get_logger().info("[GOTO] goal failed to cancel")

        self.goto_goal_handle = None

    def goto_cancel(self):
        if self.goto_goal_handle is None:
            return

        self.get_logger().warn("[GOTO] Canceling goal...")
        cancel_future = self.goto_goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self.goto_cancel_callback)
        self.pub_twist(Twist())

        return cancel_future

    def goto_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("[GOTO] goal rejected :(")
            return

        self.get_logger().info("[GOTO] goal accepted :)")
        self.goto_goal_handle = goal_handle

    def goto(self, x_, y_, theta_=None, relative=False, rotation=False, bourrage=False):
        if self.end_match.done():
            self.get_logger().warn("Match is finished, ignoring GoTo")
            return False

        if self.obstacle_stop:
            self.get_logger().warn("Obstacle ahead, ignoring GoTo")
            return False

        if theta_ is None:
            new_theta = 0.0
            self.get_logger().info("No heading provided")

            if self.currentX is None and self.currentY is None:
                self.get_logger().warn(
                    "No odometry data available. Default heading to 0"
                )
                new_theta = 0.0
            else:
                if self.team.result() == "yellow":
                    x = 2 - x_
                else:
                    x = x_
                new_theta = atan2(y_ - self.currentY, x - self.currentX)

            self.spin(new_theta)
            
            theta_ = atan2(y_ - self.currentY, x_ - self.currentX)

        # self.get_logger().info(f"GoTo ({str(x_)}, {str(y_)}, ${str(theta_)}) ")

        self.goto_goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        self.goto_goal_msg.pose.header.frame_id = "map" if not relative else "base_footprint"

        x, y, theta = 0, 0, 0

        if relative:
            x = x_
            y = y_
            theta = -theta # à vérifier pour spin relatif
        else:
            x, y, theta = self.get_pose(x_, y_, theta_)

    self.get_logger().info(f"[GOTO] goal :({str(x)}, {str(y)}, ${str(theta)}) ")
    q = quaternion_from_euler(0, 0, theta)


        self.get_logger().info(f"[GOTO] goal :({str(x)}, {str(y)}, ${str(theta)}) ")
        q = quaternion_from_euler(0, 0, theta)


        self.goto_goal_msg.pose.pose.position.x = float(x)
        self.goto_goal_msg.pose.pose.position.y = float(y)
        self.goto_goal_msg.pose.pose.orientation.x = q[0]
        self.goto_goal_msg.pose.pose.orientation.y = q[1]
        self.goto_goal_msg.pose.pose.orientation.z = q[2]
        self.goto_goal_msg.pose.pose.orientation.w = q[3]

        self.get_logger().info(f"{q[0]} {q[1]} {q[2]} {q[3]}")

        self.goto_goal_msg.rotation = rotation

        self.goto_action_client.wait_for_server()

        send_goal_future = self.goto_action_client.send_goal_async(self.goto_goal_msg)
        send_goal_future.add_done_callback(self.goto_response_callback)

        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        goal_finished_future = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(self, goal_finished_future)
        self.goto_goal_handle = None

        result = goal_finished_future.result()

        # result = success OK

        # result false -> à cause de stuck detector ou à cause de obstacle 

        if self.obstacle_stop:
            self.get_logger().warn("[GOTO] Obstacle ahead ! Wait routine...")
            # routine de retry
            pass
        else:
            if bourrage:
                # OK c'est ce qui était voulu
                return True

            # Routine de recover, on est bloqué mais pas à cause d'un obstacle
            self.get_logger().warn("[GOTO] Robot stucked ! recover routine...")
            pass

        return result

    def spin(self, yaw, relative=False):
        self.get_logger().info(f"spin")
        self.goto(self.currentX, self.currentY, yaw, relative, rotation=True)

    def forward(self, dist=0.15, bourrage=False):
        return self.goto(dist, 0, 0, relative=True, bourrage=bourrage)

    def backup(self, dist=0.15, bourrage=False):
        return self.goto(-dist, 0, 0, relative=True, bourrage=bourrage)

    def bourrageAvant(self):
        return self.forward(dist=0.5, bourrage=True)

    def bourrageArriere(self):
        return self.forward(dist=0.5, bourrage=True)


    def set_initialpose(self, x_, y_, theta_):
        x, y, theta = self.get_pose(float(x_), float(y_), float(theta_))
        q = quaternion_from_euler(0, 0, theta)

        initialpose_msg = PoseWithCovarianceStamped()
        initialpose_msg.header.stamp = self.get_clock().now().to_msg()
        initialpose_msg.header.frame_id = "map"
        initialpose_msg.pose.pose.position.x = x
        initialpose_msg.pose.pose.position.y = y
        initialpose_msg.pose.pose.orientation.x = q[0]
        initialpose_msg.pose.pose.orientation.y = q[1]
        initialpose_msg.pose.pose.orientation.z = q[2]
        initialpose_msg.pose.pose.orientation.w = q[3]

        self.pub_initialpose.publish(initialpose_msg)
        self.pub_initialpose.publish(
            initialpose_msg
        )  
        # TODO: dirty hack, need a service / action to resolve when the pos is effectively set
        rclpy.spin_once(self)

        time.sleep(0.5)

    def loop(self):
        while rclpy.ok():
            self.pub_twist.publish(Twist())
            self.get_logger().info("[STRAT] Init strategy. Wait for team...")
            rclpy.spin_until_future_complete(self, self.team)

            self.get_logger().info("[STRAT] Wait for start...")
            rclpy.spin_until_future_complete(self, self.start)

            self.get_logger().info(
                f"[STRAT] Start ! team: {self.team.result()} strategy: {str(self.strategy)}"
            )
            self.end_match_timer = self.create_timer(
                MATCH_DURATION, self.on_end_match, callback_group=self.cb_group
            )

            ######### Strategy here, written for PURPLE TEAM #########
            
            ## STRAT 0
            start_angle = 0.0
            self.set_initialpose(0.0, 0.0, radians(start_angle))

            self.forward(0.1)
            self.spin(radians(45.0))

            

            self.printScore()

            ## END STRAT 0

            ######### End strategy ##########

            self.get_logger().info("[STRAT] Strategy finished ! Ending match...")
            self.on_end_match()

            rclpy.spin_until_future_complete(self, self.end_match)

            self.pub_end_match.publish(Empty())
            self.goto_cancel()

            rclpy.spin_until_future_complete(self, self.reset)

            self.start = Future()
            self.end_match = Future()


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
