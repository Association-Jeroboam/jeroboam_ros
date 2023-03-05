import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from jrb_msgs.msg import MotionSpeedCommand


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(MotionSpeedCommand, 'speed_command', 10)
        self.timer_period = 0.05  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = MotionSpeedCommand()
        
        t = self.i * self.timer_period
        acc = -0.5
        vmax = -0.6

        t1 = vmax / acc
        t2 = t1 + 3
        t3 = t2 + t1

        if t <= 0:
            msg.right = 0.0
        elif t < t1:
            msg.right  = acc * t
        elif t < t2:
            msg.right = vmax
        elif t < t3:
            msg.right = vmax - acc*(t-t2)
        else:
            self.i = -20
            msg.right = 0.0

        self.i += 1
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()