import os
import struct
from rclpy.node import Node
import rclpy
import time
import threading
import serial
from std_msgs.msg import String, Header
from jrb_msgs.msg import MotionBoardDataFrame
from dataclasses import dataclass
import glob
import subprocess
import threading
import traceback
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

SYNCHRO_WORD = b"\xef\xbe\xad\xde"
FRAME_SCHEMA = "<Iffffffffffffffff"
FRAME_SIZE = struct.calcsize(FRAME_SCHEMA)


@dataclass
class Parameters:
    baudrate = 115200
    tty_path = "/dev/ttyACM*"


@dataclass
class State:
    serial_port = None
    data_stream_started = False
    byte_buffer = bytearray()
    uart_read_thread = None
    serial_connect_timer = None


class HardwareBridgeUart(Node):
    def __init__(self):
        super().__init__("hardware_bridge_uart")

        # Parameters
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("tty_path", "/home/pymzor/virtual-tty")

        self.parameters = Parameters()
        self.parameters.baudrate = self.get_parameter("baudrate").value
        self.parameters.tty_path = self.get_parameter("tty_path").value

        self.add_on_set_parameters_callback(self.on_update_parameters)

        # Internal state
        self.state = State()
        self.state.serial_connect_timer = self.create_timer(1, self.connect_to_serial)

        # Publishers
        self.pub_motion_board_data = self.create_publisher(
            MotionBoardDataFrame, "motion_board/uart/data", 10
        )
        self.pub_motion_board_log = self.create_publisher(
            String, "motion_board/uart/log", 10
        )

        # Subscribers
        self.sub_motion_board_message = self.create_subscription(
            String, "motion_board/message", self.motion_board_message_cb, 10
        )

    def cleanup(self):
        if self.state.uart_read_thread:
            self.state.uart_read_thread.join()

    def on_update_parameters(self, params):
        for param in params:
            if param.name == "baudrate":
                self.parameters.baudrate = param.value
            elif param.name == "tty_path":
                self.parameters.tty_path = param.value

        return SetParametersResult(successful=True)

    def connect_to_serial(self):
        ports_names = glob.glob(f"{self.parameters.tty_path}*")
        self.get_logger().info(f"Detected ports for {self.parameters.tty_path}: {ports_names}")

        for port_name in ports_names:
            try:
                if self.state.serial_connect_timer is not None:
                    self.state.serial_connect_timer.cancel()

                if self.state.uart_read_thread:
                    self.state.serial_port = None
                    self.state.uart_read_thread.join()

                self.state.serial_port = serial.Serial(
                    port_name, self.parameters.baudrate, timeout=1, write_timeout=1
                )

                self.get_logger().info(f"Connected to serial port: {port_name}")

                self.state.uart_read_thread = threading.Thread(
                    target=self.read_serial, daemon=True
                )
                self.state.uart_read_thread.start()

                return

            except Exception as err:
                self.get_logger().info(f"Could not open port: {str(err)}")

    def read_serial(self):
        while rclpy.ok() and self.state.serial_port is not None:
            try:
                if self.state.serial_port.in_waiting > 0:
                    data = self.state.serial_port.read(
                        self.state.serial_port.in_waiting
                    )

                    if not self.state.data_stream_started:
                        data_str = data.decode()
                        self.get_logger().info(data_str)
                        msg = String()
                        msg.data = data_str
                        self.pub_motion_board_log.publish(msg)
                    else:
                        ba = bytearray(data)
                        self.state.byte_buffer.extend(ba)
                        self.extract_frame()

            except Exception as err:
                self.get_logger().error(f"Could not read serial: {str(err)}")
                self.state.serial_port = None

                self.state.serial_connect_timer = self.create_timer(
                    1, self.connect_to_serial
                )
                continue

            time.sleep(0.01)

    def close_serial(self):
        if self.state.serial_port is None:
            return

        try:
            self.state.serial_port.close()
            self.get_logger().info("Serial port closed.")
        except Exception as err:
            self.get_logger().info(f"Could not close serial port: {str(err)}")

    def extract_frame(self):
        idx = self.state.byte_buffer.find(SYNCHRO_WORD)

        if idx > -1:
            if len(self.state.byte_buffer) >= idx + len(SYNCHRO_WORD) + FRAME_SIZE:
                frame = self.state.byte_buffer[
                    idx + len(SYNCHRO_WORD) : idx + len(SYNCHRO_WORD) + FRAME_SIZE
                ]
                self.state.byte_buffer = self.state.byte_buffer[
                    idx + len(SYNCHRO_WORD) + FRAME_SIZE :
                ]

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

    def motion_board_message_cb(self, message):
        message = message.data

        if self.state.serial_port is None:
            self.get_logger().warn(
                f"Serial port is not open, ignoring message: {message}"
            )
            return

        if message == "data_stream start":
            self.state.data_stream_started = True
        elif message == "data_stream stop":
            self.state.data_stream_started = False

        try:
            self.sio.emit("message", message + "\r\n")
            self.get_logger().info(f"[SEND] {message}")
        except Exception as err:
            self.get_logger().error(f"Could not send command: {err}")

    def run_shell_cmd(self, cmd):
        self.get_logger().info(f"Running shell cmd: {cmd}")

        proc = subprocess.run([cmd], stdout=subprocess.PIPE, universal_newlines=True)

        self.get_logger().info(f"[{cmd!r} exited with {proc.returncode}]")

        if proc.stdout:
            self.get_logger().info(f"[stdout]\n{proc.stdout.decode()}")

        if proc.stderr:
            self.get_logger().info(f"[stderr]\n{proc.stderr.decode()}")

    def flash(self, path):
        self.get_logger().info("Flashing...")

        if not os.path.exists(path):
            self.get_logger().info(f"Error: path {path} does not exists")
            return

        with open(path, "rb") as f:
            file_data = f.read()

        process = subprocess.run(
            ["mkdir -p build"], stdout=subprocess.PIPE, universal_newlines=True
        )

        self.run_shell_cmd("mkdir -p build && rm -rf ./build/build.tar.gz")
        self.run_shell_cmd(f"cp {path} build/build.tar.gz")
        self.run_shell_cmd("cd build && tar -xvf build.tar.gz && cd build")
        self.run_shell_cmd(
            'openocd -s ./cfg -c "set BIN_FILE build/MotionBoard.bin" -f flash.cfg'
        )


def main(args=None):
    rclpy.init(args=args)
    node = HardwareBridgeUart()

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
        node.cleanup()


if __name__ == "__main__":
    main()
