
import threading
import time

import rclpy
import serial
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray


class RadioCtrlNode(Node):
    def __init__(self):
        super().__init__('radio_ctrl_node')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('serial_timeout', 0.25)
        self.declare_parameter('reconnect_interval', 2.0)

        self.port = str(self.get_parameter('port').value)
        self.baud = int(self.get_parameter('baud').value)
        self.stale_timeout = float(
            self.get_parameter('serial_timeout').value
        )
        self.reconnect_interval = max(
            0.5,
            float(self.get_parameter('reconnect_interval').value),
        )

        self.publisher = self.create_publisher(
            Twist, '/teleop/radio_cmd', 10
        )
        self.channels_publisher = self.create_publisher(
            UInt16MultiArray, '/teleop/radio_channels', 10
        )

        self.lock = threading.Lock()
        self.last_valid_time = 0.0

        self.stop_event = threading.Event()
        self.serial_port = None
        self.last_warning_time = 0.0

        self.reader_thread = threading.Thread(
            target=self.read_serial,
            daemon=True,
        )
        self.reader_thread.start()

        self.timer = self.create_timer(0.02, self.publish_command)

        self.get_logger().info(
            f'Radio input: {self.port} at {self.baud} baud'
        )

    def warn_throttled(self, message):
        now = time.monotonic()
        if now - self.last_warning_time >= 5.0:
            self.get_logger().warning(message)
            self.last_warning_time = now

    def read_serial(self):
        while not self.stop_event.is_set() and rclpy.ok():
            port = None

            try:
                self.warn_throttled(
                    f'Opening serial port {self.port}'
                )
                port = serial.Serial(
                    port=self.port,
                    baudrate=self.baud,
                    timeout=0.1,
                )
                self.serial_port = port
                self.get_logger().info(
                    f'Serial connected: {self.port}'
                )
                self.last_warning_time = 0.0

                while (
                    not self.stop_event.is_set()
                    and rclpy.ok()
                    and port.is_open
                ):
                    raw = port.readline()
                    if not raw:
                        continue

                    parts = (
                        raw.decode('ascii', errors='ignore')
                        .strip()
                        .split(',')
                    )

                    # RC,valid,ch1,ch2,...,ch16 = 18 fields
                    if len(parts) != 18 or parts[0] != 'RC':
                        continue

                    try:
                        valid = int(parts[1])
                        channels = [
                            int(value) for value in parts[2:18]
                        ]
                    except ValueError:
                        continue

                    if valid not in (0, 1):
                        continue

                    if any(
                        value < 0 or value > 2047
                        for value in channels
                    ):
                        continue

                    channel_msg = UInt16MultiArray()
                    channel_msg.data = channels

                    if valid == 1:
                        self.channels_publisher.publish(channel_msg)
                        with self.lock:
                            self.last_valid_time = time.monotonic()
                    else:
                        # Publish zeros when the receiver reports
                        # that the RC signal is invalid.
                        channel_msg.data = [0] * 16
                        self.channels_publisher.publish(channel_msg)
                        with self.lock:
                            self.last_valid_time = 0.0

            except (serial.SerialException, OSError) as exc:
                self.reset_signal()
                self.warn_throttled(
                    f'Serial disconnected/unavailable: {exc}'
                )

            finally:
                self.reset_signal()

                if port is not None:
                    try:
                        if port.is_open:
                            port.close()
                    except (serial.SerialException, OSError):
                        pass

                if self.serial_port is port:
                    self.serial_port = None

            if not self.stop_event.is_set() and rclpy.ok():
                self.stop_event.wait(self.reconnect_interval)

    def reset_signal(self):
        with self.lock:
            self.last_valid_time = 0.0

    def publish_command(self):
        # Movement commands remain disabled while we verify all
        # 16 incoming RC channels.
        msg = Twist()
        self.publisher.publish(msg)

        now = time.monotonic()
        with self.lock:
            if (
                self.last_valid_time > 0.0
                and now - self.last_valid_time > self.stale_timeout
            ):
                self.last_valid_time = 0.0

    def destroy_node(self):
        self.stop_event.set()

        port = self.serial_port
        if port is not None:
            try:
                if port.is_open:
                    port.close()
            except (serial.SerialException, OSError):
                pass

        if self.reader_thread.is_alive():
            self.reader_thread.join(timeout=1.0)

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = RadioCtrlNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
