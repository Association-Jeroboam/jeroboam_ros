import traceback
import RPi.GPIO as GPIO
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from rclpy.timer import Timer
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, FloatingPointRange
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

STARTER_PIN = 11
TEAM_PIN = 12
STRATEGY_PIN = 13


class RaspiGpio(Node):
    def __init__(self):
        super().__init__("raspi_gpio")
        self.get_logger().info(f"{self.get_name()} started")

        self.init_gpio()

        latchedQoS = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=1,
        )

        self.pub_starter = self.create_publisher(Bool, "hardware/starter", latchedQoS)
        self.pub_team = self.create_publisher(String, "hardware/team", latchedQoS)
        self.pub_strategy = self.create_publisher(Bool, "hardware/strategy", latchedQoS)

        self.declare_parameter(
            "starter_polling_rate",
            5.0,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                floating_point_range=[
                    FloatingPointRange(from_value=1.0, to_value=10.0, step=0.1)
                ],
            ),
        )

        self.declare_parameter(
            "team_polling_rate",
            2.0,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                floating_point_range=[
                    FloatingPointRange(from_value=1.0, to_value=10.0, step=0.1)
                ],
            ),
        )

        self.declare_parameter(
            "strategy_polling_rate",
            2.0,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                floating_point_range=[
                    FloatingPointRange(from_value=1.0, to_value=10.0, step=0.1)
                ],
            ),
        )

        self.starter_polling_rate = self.get_parameter("starter_poling_rate").value
        self.team_polling_rate = self.get_parameter("team_poling_rate").value
        self.strategy_polling_rate = self.get_parameter("strategy_poling_rate").value

        self.last_starter_value = None
        self.last_team_value = None
        self.last_strategy_value = None

        self.starter_polling_timer = self.create_timer(
            1 / self.starter_polling_rate, self.on_starter_polling
        )
        self.team_polling_timer = self.create_timer(
            1 / self.team_polling_rate, self.on_team_polling
        )
        self.strategy_polling_timer = self.create_timer(
            1 / self.strategy_polling_rate, self.on_strategy_polling
        )

        self.add_on_set_parameters_callback(self.on_update_parameters)

    def on_update_parameters(self, params):
        for param in params:
            if param.name == "starter_polling_rate":
                self.starter_polling_rate = param.value
                self.starter_polling_timer.timer_period_ns = (
                    1 / self.starter_polling_rate
                )
            elif param.name == "team_polling_name":
                self.team_polling_rate = param.value
                self.team_polling_timer.timer_period_ns = 1 / self.team_polling_rate
            elif param.name == "strategy_polling_name":
                self.strategy_polling_rate = param.value
                self.strategy_polling_timer.timer_period_ns = (
                    1 / self.strategy_polling_rate
                )

        self.get_logger().info("Params updated")

        return SetParametersResult(successful=True)

    def init_gpio(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(STARTER_PIN, GPIO.IN)
        GPIO.setup(TEAM_PIN, GPIO.IN)
        GPIO.setup(STRATEGY_PIN, GPIO.IN)

    def on_starter_polling(self):
        starter_value = bool(GPIO.input(STARTER_PIN))

        if starter_value != self.last_starter_value:
            self.last_starter_value = starter_value
            self.pub_starter.publish(Bool(data=starter_value))

    def on_team_polling(self):
        team_value = bool(GPIO.input(TEAM_PIN))

        if team_value != self.last_team_value:
            self.last_team_value = team_value
            self.pub_team.publish(Bool(data=team_value))

    def on_strategy_polling(self):
        strategy_value = bool(GPIO.input(STRATEGY_PIN))

        if strategy_value != self.last_strategy_value:
            self.last_strategy_value = strategy_value
            self.pub_strategy.publish(Bool(data=strategy_value))


def main(args=None):
    rclpy.init(args=args)

    node = RaspiGpio()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped cleanly")
    except Exception:
        print("Error while stopping the node")
        print(traceback.format_exc())
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
