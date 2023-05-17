import traceback
import rclpy
from rclpy.task import Future
from jrb_control.robot_navigator import BasicNavigator
from std_msgs.msg import Bool, Int8, Int16, UInt16, UInt8, String, Empty
from geometry_msgs.msg import PoseArray, Pose2D
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

MATCH_DURATION = 20  # s


class Strategy(BasicNavigator):
    def __init__(self, node_name):
        super().__init__(node_name=node_name)

        self.latched_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.sub_start = self.create_subscription(
            Bool,
            "hardware/starter",
            self.on_starter,
            1,
        )
        self.sub_team = self.create_subscription(
            String,
            "hardware/team",
            self.on_team,
            self.latched_qos,
        )
        self.sub_strategy = self.create_subscription(
            Bool,
            "hardware/strategy",
            self.on_strategy,
            self.latched_qos,
        )
        self.sub_obstacle = self.create_subscription(
            PoseArray,
            "obstacle_detected",
            self.on_obstacle_detected,
            10,
        )

        self.pub_end_match = self.create_publisher(Empty, "strategy/end_match", 1)

        self.start_match_future = Future()
        self.end_match_future = Future()
        self.reset_future = Future()
        self.team_future = Future()
        self.strategy = None
        self.end_match_timer = None
        self.obstacle_stop = False
        self.start_time = None

    def on_starter(self, msg: Bool):
        value = msg.data
        self.info("starter : " + str(value))

        if value and not self.start_match_future.done():
            self.info("start match")
            self.start_match_future.set_result("set")
            self.reset_future = Future()

            self.end_match_timer = self.create_timer(MATCH_DURATION, self.on_end_match)
            self.start_time = self.get_clock().now()

        # Reset so we can start match again when it is finished
        if not value and self.start_match_future.done():
            self.info("start match")
            self.reset_future.set_result("set")
            self.start_match_future = Future()
            self.end_match_future = Future()

    def on_team(self, msg: String):
        if not self.start_match_future.done():
            self.team_future.set_result(msg.data)

    def on_strategy(self, msg: Bool):
        self.strategy = msg.data

    def on_obstacle_detected(self, msg: PoseArray):
        if len(msg.poses) > 0:
            self.warn("Obstacle detected ! Cancelling nav goal")
            self.cancelNavTask()

    def on_end_match(self):
        self.info("End match timer")
        self.end_match_future.set_result("set")

        if self.end_match_timer is not None:
            self.end_match_timer.cancel()

        self.stop()
        self.pub_end_match.publish(Empty())

    def waitForMatchToStart(self):
        rclpy.spin_until_future_complete(self, self.start_match_future)

    def waitForMatchToEnd(self):
        rclpy.spin_until_future_complete(self, self.end_match_future)

    def waitForTeamSelect(self):
        rclpy.spin_until_future_complete(self, self.team_future)

    def waitForReset(self):
        rclpy.spin_until_future_complete(self, self.reset_future)

    def isMatchFinished(self):
        if not rclpy.ok():
            return True

        if not self.end_match_future:
            return False

        rclpy.spin_until_future_complete(self, self.end_match_future, timeout_sec=0.10)

        if self.end_match_future.result():
            return True
        else:
            return False

    def getTeam(self):
        return self.team_future.result()

    def getPose(self, pose: Pose2D) -> Pose2D:
        team = self.getTeam()

        # yellow = green = droite
        # purple = blue = gauche
        if team == "purple":
            return pose
        elif team == "yellow":
            ret = Pose2D()
            ret.x = 2 - pose.x
            ret.y = pose.y
            ret.theta = (360.0 - ret.theta) % 360.0

            return ret
        else:
            raise Exception("Team has not been selected")

    def goToPose(self, pose: Pose2D, relative=False, blocking=True):
        if self.isMatchFinished():
            return

        pose_transformed = self.getPose(pose)
        super().goToPose(pose_transformed, relative, blocking)

    def setInitialPose(self, initial_pose: Pose2D):
        pose_transformed = self.getPose(initial_pose)
        super().setInitialPose(pose_transformed)

    def secSinceStartMatch(self) -> float:
        duration = self.get_clock().now() - self.start_time
        sec = duration.nanoseconds / 1e9
        return sec

    def loop(self):
        while rclpy.ok():
            try:
                self.doStrategy()
                self.stop()
                self.info("Strategy finished !")
            except Exception:
                self.error("Error while executing strategy")
                self.error(traceback.format_exc())

            self.info("Wait for reset...")
            self.waitForReset()
