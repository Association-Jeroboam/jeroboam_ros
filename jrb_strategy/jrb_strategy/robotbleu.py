import rclpy
from .strategy import Strategy
from std_msgs.msg import Bool, Int8, Int16, UInt16, UInt8, String
from geometry_msgs.msg import Pose2D

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
        if self.isMatchFinished():
            return
        self.info("turbine stop")
        self.pub_turbine_speed.publish(UInt16(data=0))

    def funnyAction(self):
        # TODO
        pass

    def stop(self):
        super().stop()
        self.rollerStop()
        self.turbineStop()

    def doStrategy(self):
        self.stop()

        self.info("Init strategy. Wait for team...")
        self.waitForTeamSelect()

        self.info("Wait for start...")
        self.waitForMatchToStart()

        self.info(f"Start ! team: {self.getTeam()}")

        self.goToPose(Pose2D(x=0.5, y=0.0, theta=0.0))
        self.printPose()

        self.forward()
        self.printPose()

        self.backup()
        self.printPose()

        self.spin(45)
        self.printPose()

        self.spin(-45)
        self.printPose()

        self.spin(0)
        self.printPose()

        self.info("Wait for match to end")
        self.waitForMatchToEnd()

        self.info("Sleep 2s before funny action")
        self.sleep(2)

        self.info("FUNNY ACTION LOL")


def main():
    rclpy.init()

    bot = RobotBleu()
    bot.loop()

    # from ptpython.repl import embed

    # embed(globals(), locals())


if __name__ == "__main__":
    main()
