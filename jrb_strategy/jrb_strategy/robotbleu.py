import rclpy
from .strategy import Strategy
from std_msgs.msg import Bool, Int8, Int16, UInt16, UInt8, String
from geometry_msgs.msg import Pose2D, PoseArray

ROLL_HEIGHT_ACTION_TIME = 1  # s


class RobotBleu(Strategy):
    def __init__(self):
        super().__init__(node_name="robotbleu_navigator")

        self.pub_roll_height = self.create_publisher(Int16, "hardware/roll/height", 10)
        self.pub_roll_speed = self.create_publisher(Int8, "hardware/roll/speed", 10)
        self.pub_turbine_speed = self.create_publisher(
            UInt16, "hardware/turbine/speed", 10
        )

    def rollerUp(self):
        if self.isMatchFinished():
            return
        self.info("roller up")
        self.pub_roll_height.publish(Int16(data=2))
        self.sleep(ROLL_HEIGHT_ACTION_TIME)

    def rollerMiddle(self):
        if self.isMatchFinished():
            return
        self.info("roller middle")
        self.pub_roll_height.publish(Int16(data=1))
        self.sleep(ROLL_HEIGHT_ACTION_TIME)

    def rollerLow(self):
        if self.isMatchFinished():
            return
        self.info("roller low")
        self.pub_roll_height.publish(Int16(data=0))
        self.sleep(ROLL_HEIGHT_ACTION_TIME)

    def rollerIn(self):
        if self.isMatchFinished():
            return
        self.info("roller in")
        self.pub_roll_speed.publish(Int8(data=1))

    def rollerOut(self):
        if self.isMatchFinished():
            return
        self.info("roller out")
        self.pub_roll_speed.publish(Int8(data=-1))

    def rollerStop(self):
        if self.isMatchFinished():
            return
        self.info("roller stop")
        self.pub_roll_speed.publish(Int8(data=0))

    def turbineStart(self):
        if self.isMatchFinished():
            return
        self.info("turbine start")
        self.pub_turbine_speed.publish(UInt16(data=1))

    def turbineStartFullSpeed(self):
        if self.isMatchFinished():
            return
        self.info("turbine start FULLSPEED")
        self.pub_turbine_speed.publish(UInt16(data=2))

    def turbineStop(self):
        self.info("turbine stop")
        self.pub_turbine_speed.publish(UInt16(data=0))

    def funnyAction(self):
        # TODO
        pass

    def stop(self):
        super().stop()
        self.rollerStop()
        self.turbineStop()

    def on_obstacle_detected(self, msg: PoseArray):
        # pass
        super().on_obstacle_detected(msg)

    def doStrategy(self):
        self.stop()

        # self.rollerUp()
        # self.wait_seconds(2)
        # self.rollerMiddle()
        # self.wait_seconds(2)
        # self.rollerLow()

        # self.wait_seconds(2)
        # self.rollerIn()
        # self.wait_seconds(2)
        # self.rollerOut()
        # self.wait_seconds(2)
        # self.rollerStop()

        self.info("Init strategy ROBOTBLEU. Wait for team...")
        self.waitForTeamSelect()

        self.info("Wait for start...")
        self.waitForMatchToStart()

        self.info(f"Start ! team: {self.getTeam()}")

        # Set initial pause
        self.setInitialPose(Pose2D(x=0.25, y=2.75, theta=90.0))

        # Set initial pause
        
        self.setInitialPose(Pose2D(x=0.33, y=2.67, theta=0.0))
        self.printPose()

        # got to push disks
        self.goTo(Pose2D(x=0.56, y=2.20, theta=-90.0))

        self.goToPose(Pose2D(x=0.27, y=1.83, theta=90.0))

        #ready to push disks

        self.goToPose(Pose2D(x=0.27, y=2.57 , theta=-90.0))

        self.goToPose(Pose2D(x=0.27, y=2.4 , theta=0.0))

        #ready to go fetch cherries

        self.goToPose(Pose2D(x=0.72, y=2.57 , theta=90.0))

        self.goToPose(Pose2D(x=0.72, y=2.77 , theta=0.0))
        self.goToPose(Pose2D(x=0.72, y=2.77 , theta=0.0))
        #READY TO FECTH
        self.rollerMiddle()
        self.rollerIn()

        self.goToPose(Pose2D(x=0.89, y=2.81 , theta=0.0))

        self.wait_seconds(4)

        self.rollerOut()

        self.goToPose(Pose2D(x=0.5, y=2.56 , theta=113.0))

        self.turbineStartFullSpeed()
        self.wait_seconds(10)
        self.turbineStop()

        self.on_end_match()

        self.backup(0.5)    

        # spin
        self.forward(0.5)
        self.spin(90.0)
        self.forward(0.5)
        self.spin(180)
        self.forward(0.5)
        self.spin(-90.0)
        self.forward(0.5)
        self.spin(0.0)

        self.backup(0.5)
        self.spin(90.0)
        self.backup(0.5)
        self.spin(180)
        self.backup(0.5)
        self.spin(-90.0)
        self.backup(0.5)
        self.spin(0.0)

        # Backup
        # self.backup(dist=0.5)
        self.printPose()

        # Shoot balls for 10s
        # self.turbineStartFullSpeed()
        # self.wait_seconds(10)
        # self.turbineStop()

        self.on_end_match()

        # self.goToPose(Pose2D(x=0.5, y=0.0, theta=0.0))
        # self.printPose()

        # self.forward(dist=0.3)
        # self.printPose()

        # self.backup(dist=0.3)
        # self.printPose()


def main():
    rclpy.init()

    bot = RobotBleu()
    bot.loop()

    # from ptpython.repl import embed

    # embed(globals(), locals())


if __name__ == "__main__":
    main()
