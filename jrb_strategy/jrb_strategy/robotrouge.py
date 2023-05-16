import rclpy
from .strategy import Strategy


class RobotRouge(Strategy):
    def __init__(self):
        super().__init__(node_name="robotrouge_navigator")

    # TODO


def main():
    rclpy.init()

    bot = RobotRouge()

    from ptpython.repl import embed

    embed(globals(), locals())


if __name__ == "__main__":
    main()
