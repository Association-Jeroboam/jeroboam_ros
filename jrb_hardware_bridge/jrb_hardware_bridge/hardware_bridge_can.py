import traceback
import os
import sys
from struct import pack, unpack
from rclpy.node import Node
import rclpy
import time
from std_msgs.msg import String, Header
from nav_msgs.msg import Odometry
from jrb_msgs.msg import MotionBoardDataFrame
from geometry_msgs.msg import Quaternion
import can
import threading
from tf_transformations import quaternion_from_euler, euler_from_quaternion

def create_quaternion_msg_from_yaw(yaw):
   quat = quaternion_from_euler(0, 0, yaw)
   return Quaternion(**dict(zip(['x', 'y', 'z', 'w'], quat)))

FRAMES = {
    # Pliers Command (ID, state)
    0x11: 'TODO',
    # Slider Command (Position, ID)
    0x12: 'TODO',
    # Color sensor (ID, R, G, B)
    0x16: 'TODO',
    # Action done (ActionID, BoardID)
    0x1F: 'TODO',
    # Position current (x, y, theta)
    0x20: '<hhf',
    # Set position (goal x, y, theta)
    0x21: '<hhf',
    # Speed current (v, w)
    0x22: '<ff',
    # Set velocity goal (v, w)
    0x23: '<ff',
    # Set odometry (x, y, theta)
    0x40: '<hhf',
    # Robot reset
    0x666: '',
    # Calibrate actuators
    0x666: ''
} 

CALLBACK_BY_FRAME_ID = {}

# This decorator registers the function in CALLBACK_BY_FRAME_ID
def can_callback(frame_id):
    def decorator(func):
        CALLBACK_BY_FRAME_ID[frame_id] = func
        return func
    
    return decorator

def mm_to_m(mm):
    return mm / 1000

def m_to_mm(m):
    return round(m*1000)

def create_can_frame(frame_id, data):
    data_struct = FRAMES.get(frame_id, None)

    if data_struct is None:
        raise Exception(f'Unknown data struct for frame_id {frame_id}')

    frame = pack(data_struct, *data)

    return can.Message(arbitration_id=frame_id, data=frame)


class HardwareBridgeCan(Node):
    def __init__(self):
        super().__init__('hardware_bridge_can')
        self.get_logger().info('init')

        self.declare_parameter("can_interface", "vcan0")
        self.can_interface = self.get_parameter("can_interface").value

        self.init_can_bus()
        self.reset_state()

        self.pub_odometry = self.create_publisher(Odometry, 'motion_board/odometry', 10)

        self.sub_set_odometry = self.create_subscription(Odometry, 'motion_board/set_odometry', self.set_odometry_cb, 10)


    def cleanup(self):
        if self.can_read_thread:
            self.can_read_thread.join()

    def reset_state(self):
        self.state = {
            'pos': None,
            'vel': None
        }

    def publish_odometry(self):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'

        odom.pose.pose.position.x = mm_to_m(self.state['pos'][0])
        odom.pose.pose.position.y = mm_to_m(self.state['pos'][1])
        odom.pose.pose.position.z = 0.0
        
        odom.pose.pose.orientation = create_quaternion_msg_from_yaw(self.state['pos'][2])

        odom.twist.twist.linear.x = self.state['vel'][0]
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0

        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = self.state['vel'][1]
        
        self.pub_odometry.publish(odom)

    def init_can_bus(self):
        self.can0 = can.interface.Bus(channel=self.can_interface, bustype='socketcan')
        self.get_logger().info('can bus init, run thread...')
        self.can_read_thread = threading.Thread(target=self.extract_frame, daemon=True)
        self.can_read_thread.start()

    def extract_frame(self):
        self.get_logger().info('thread start')

        while rclpy.ok():
            msg = self.can0.recv(1)

            # Timeout
            if not msg:
                self.get_logger().info('Can timeout')
                continue


            callback = CALLBACK_BY_FRAME_ID.get(msg.arbitration_id, None)

            if callback is not None:
                frame = FRAMES.get(msg.arbitration_id, None)

                try:
                    data = unpack(frame, msg.data)
                    callback(self, data)
                except Exception as err:
                    self.get_logger().error(f'Error while calling callback {callback.__name__}')
                    self.get_logger().error(traceback.format_exc())
                    continue
            else:
                self.get_logger().error(f'Unknown frame id: {hex(msg.arbitration_id)}')

        self.get_logger().info('thread stop')

    @can_callback(0x20)
    def on_can_pos(self, data):
        self.state['pos'] = data
        
        if self.state['vel'] is not None:
            self.publish_odometry()
            self.reset_state()


    @can_callback(0x22)
    def on_can_vel(self, data):
        self.state['vel'] = data
        
        if self.state['pos'] is not None:
            self.publish_odometry()
            self.reset_state()

    def set_odometry_cb(self, msg):
        x, y = m_to_mm(msg.pose.pose.position.x), m_to_mm(msg.pose.pose.position.y)
        _, _, yaw = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])

        can_frame = create_can_frame(0x40, (x, y, yaw))
        self.can0.send(can_frame)

def main(args=None):
    rclpy.init(args=args)
    node = HardwareBridgeCan()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped cleanly')
    except Exception:
        print('Error while stopping the node')
        print(traceback.format_exc())
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown() 
        node.cleanup()

if __name__ == '__main__':
    main()
