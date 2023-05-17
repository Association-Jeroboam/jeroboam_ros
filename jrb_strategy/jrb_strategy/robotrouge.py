import rclpy
from .strategy import Strategy
from std_msgs.msg import Bool, Int8, Int16, UInt16, UInt8, String
from geometry_msgs.msg import Pose2D, PoseArray


class RobotRouge(Strategy):
    def __init__(self):
        super().__init__(node_name="robotrouge_navigator")

    def on_obstacle_detected(self, msg: PoseArray):
        # pass
        super().on_obstacle_detected(msg)

    def doStrategy(self):
        self.stop()

        self.info("Init strategy ROBOTROUGE. Wait for team...")
        self.waitForTeamSelect()

        self.info("Wait for start...")
        self.waitForMatchToStart()

        self.info(f"Start ! team: {self.getTeam()}")
        
        # Set initial pause
        self.setInitialPose(Pose2D(x=0.25, y=2.75, theta=90.0))

        # Backup
        self.backup(dist=0.5)
        self.printPose()

def main():
    rclpy.init()

    bot = RobotRouge()
    bot.loop()

    # from ptpython.repl import embed

    # embed(globals(), locals())


if __name__ == "__main__":
    main()
