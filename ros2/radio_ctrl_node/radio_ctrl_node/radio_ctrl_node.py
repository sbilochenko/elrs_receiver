
import threading
import time

import rclpy
import serial
from geometry_msgs.msg import Twist
from rclpy.node import Node


class RadioCtrlNode(Node):
    def __init__(self):
        super().__init__('radio_ctrl_node')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('max_linear_speed', 0.3)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('serial_timeout', 0.25)
        self.declare_parameter('reconnect_interval', 2.0)

        self.port = str(self.get_parameter('port').value)
        self.baud = int(self.get_parameter('baud').value)
        self.max_linear = float(
            self.get_parameter('max_linear_speed').value)
        self.max_angular = float(
            self.get_parameter('max_angular_speed').value)
        self.stale_timeout = float(
            self.get_parameter('serial_timeout').value)
        self.reconnect_interval = max(
            0.5,
            float(self.get_parameter('reconnect_interval').value),
        )

        self.publisher = self.create_publisher(
            Twist, '/teleop/radio_cmd', 10)

        self.lock = threading.Lock()
        self.linear = 0.0
        self.angular = 0.0
        self.last_valid_time = 0.0

        self.stop_event = threading.Event()
        self.serial_port = None
        self.last_warning_time = 0.0

        self.reader_thread = threading.Thread(
            target=self.read_serial, daemon=True)
        self.reader_thread.start()

        self.timer = self.create_timer(0.02, self.publish_command)
        self.get_logger().info(
            f'Radio input: {self.port} at {self.baud} baud')

    def reset_command(self):
        with self.lock:
            self.linear = 0.0
            self.angular = 0.0
            self.last_valid_time = 0.0

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
                    f'Opening serial port {self.port}')
                port = serial.Serial(
                    port=self.port,
                    baudrate=self.baud,
                    timeout=0.1,
                )
                self.serial_port = port
                self.get_logger().info(
                    f'Serial connected: {self.port}')
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

                    if len(parts) != 4 or parts[0] != 'RC':
                        continue

                    try:
                        linear = float(parts[1])
                        angular = float(parts[2])
                        valid = int(parts[3])
                    except ValueError:
                        continue

                    if valid not in (0, 1):
                        continue
                    if not (-1.0 <= linear <= 1.0):
                        continue
                    if not (-1.0 <= angular <= 1.0):
                        continue

                    with self.lock:
                        if valid == 1:
                            self.linear = linear
                            self.angular = angular
                            self.last_valid_time = time.monotonic()
                        else:
                            self.linear = 0.0
                            self.angular = 0.0
                            self.last_valid_time = 0.0

            except (serial.SerialException, OSError) as exc:
                self.reset_command()
                self.warn_throttled(
                    f'Serial disconnected/unavailable: {exc}')

            finally:
                self.reset_command()
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

    def publish_command(self):
        msg = Twist()
        now = time.monotonic()

        with self.lock:
            fresh = (
                self.last_valid_time > 0.0
                and now - self.last_valid_time <= self.stale_timeout
            )

            if fresh:
                msg.linear.x = self.linear * self.max_linear
                msg.angular.z = -self.angular * self.max_angular
            else:
                self.linear = 0.0
                self.angular = 0.0
                self.last_valid_time = 0.0

        self.publisher.publish(msg)

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
