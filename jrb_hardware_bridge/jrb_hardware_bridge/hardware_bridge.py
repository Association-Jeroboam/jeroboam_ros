import socketio
import os
import sys
import struct
from rclpy.node import Node
from rclpy.time import Time
import rclpy
import time
from std_msgs.msg import String, Header
from jrb_msgs.msg import MotionBoardDataFrame

SYNCHRO_WORD = b'\xef\xbe\xad\xde'
FRAME_SCHEMA = '<Iffffffffffffffff'
FRAME_SIZE = struct.calcsize(FRAME_SCHEMA)

class HardwareBridge(Node):
    def __init__(self):
        super().__init__('hardware_bridge')
        self.data_stream_started = False
        self.byte_buffer = bytearray()

        self.sio = socketio.Client()

    def sio_connect(self):
        try:
            self.sio.connect(f'http://robotrouge.local:8080')
        except Exception:
            return False

        self.pub_motion_board_data = self.create_publisher(MotionBoardDataFrame, 'motion_board/data', 10)
        self.pub_motion_board_log = self.create_publisher(String, 'motion_board/log', 10)

        self.sub_motion_board_message = self.create_subscription(String, 'motion_board/message', self.motion_board_message_cb, 10)

        self.sio.on('connect', self.sio_connect_cb)
        self.sio.on('disconnect', self.sio_disconnect_cb)
        self.sio.on('message', self.sio_message_cb)

        return True

    def extract_frame(self):
        idx = self.byte_buffer.find(SYNCHRO_WORD)
        if idx > -1:
            if len(self.byte_buffer) >= idx + len(SYNCHRO_WORD) + FRAME_SIZE:
                frame = self.byte_buffer[idx+len(SYNCHRO_WORD):idx+len(SYNCHRO_WORD)+FRAME_SIZE]
                self.byte_buffer = self.byte_buffer[idx+len(SYNCHRO_WORD)+FRAME_SIZE:]
                # TODO : unpack from
                data = struct.unpack_from(FRAME_SCHEMA, bytes(frame))

                # Publish data
                data_msg = MotionBoardDataFrame()
                data_msg.header = Header()
                data_msg.counter = data[0]
                data_msg.speed_left_current = data[1]
                data_msg.speed_left_goal = data[2]
                data_msg.speed_right_current = data[3]
                data_msg.speed_right_goal = data[4]
                data_msg.speed_angular_setpoint = data[6]
                data_msg.speed_angular_current = data[5]
                data_msg.speed_linear_setpoint = data[8]
                data_msg.speed_linear_current = data[7]
                data_msg.angle_setpoint = data[11]
                data_msg.angle_modulo = data[9]
                data_msg.angle_absolute = data[10]
                data_msg.distance_error = data[12]
                data_msg.position_x = data[13]
                data_msg.position_y = data[14]
                data_msg.pwm_left = data[15]
                data_msg.pwm_right = data[16]

                self.pub_motion_board_data.publish(data_msg)

                return data

        return None


    def sio_connect_cb(self):
        self.get_logger().info('Connected to server.')

    def sio_message_cb(self, data):
        if not self.data_stream_started:
            try:
                message = data.decode()
                # self.get_logger().info(f"[RECV] {message}")

                # Publish
                msg = String()
                msg.data = message

                self.pub_motion_board_log.publish(msg)
            except UnicodeDecodeError:
                self.data_stream_started = True
        else:
            self.byte_buffer.extend(bytearray(data))

            data = self.extract_frame()
            while data is not None:
                # self.get_logger().info(f"[RECV] {data}")
                data = self.extract_frame()

    def sio_disconnect_cb(self):
        self.get_logger().info('Disconnected from server.')

    def motion_board_message_cb(self, message):
        message = message.data

        if message == "data_stream start":
            data_stream_started = True
        elif message == "data_stream stop":
            data_stream_started = False

        try:
            self.sio.emit('message', message + '\r\n')
            self.get_logger().info(f'[SEND] {message}')
        except Exception as err:
            self.get_logger().error(f"Could not send command: {err}")

    def flash(self, path):
        self.get_logger().info("Flashing...")

        if not os.path.exists(path):
            self.get_logger().info(f"Error: path {path} does not exists")
            return

        with open(path, 'rb') as f:
            file_data = f.read()

        def flash_callback(*res):
            exit_code, stdout, stderr = res
            if not exit_code:
                self.get_logger().info("Success")
            else:
                self.get_logger().error(f"Flash error: {stdout}")

        sio.emit('flash', file_data, callback=flash_callback)

def main(args=None):
    rclpy.init(args=args)
    hardwareBridge = HardwareBridge()

    is_connected = hardwareBridge.sio_connect()
    while not is_connected:
        hardwareBridge.get_logger().error("Could not connect to socket io server. Retrying in 3s")
        time.sleep(3)
        is_connected = hardwareBridge.sio_connect()

    rclpy.spin(hardwareBridge)


if __name__ == '__main__':
    main()