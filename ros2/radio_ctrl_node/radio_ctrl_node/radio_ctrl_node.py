
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial


class RadioCtrlNode(Node):
    def __init__(self):
        super().__init__('radio_ctrl_node')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('max_linear_speed', 0.3)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('serial_timeout', 0.25)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        self.max_linear = float(
            self.get_parameter('max_linear_speed').value)
        self.max_angular = float(
            self.get_parameter('max_angular_speed').value)
        self.stale_timeout = float(
            self.get_parameter('serial_timeout').value)

        self.publisher = self.create_publisher(
            Twist, '/teleop/radio_cmd', 10)

        self.lock = threading.Lock()
        self.linear = 0.0
        self.angular = 0.0
        self.last_valid_time = 0.0

        self.serial_port = serial.Serial(
            port=port, baudrate=baud, timeout=0.1)

        self.reader_thread = threading.Thread(
            target=self.read_serial, daemon=True)
        self.reader_thread.start()

        self.timer = self.create_timer(0.02, self.publish_command)
        self.get_logger().info(f'Radio input: {port} at {baud} baud')

    def read_serial(self):
        while rclpy.ok() and self.serial_port is not None:
            try:
                raw = self.serial_port.readline()
                if not raw:
                    continue

                parts = raw.decode('ascii', errors='ignore').strip().split(',')
                if len(parts) != 4 or parts[0] != 'RC':
                    continue

                linear = float(parts[1])
                angular = float(parts[2])
                valid = int(parts[3])

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

            except (ValueError, serial.SerialException) as exc:
                with self.lock:
                    self.linear = 0.0
                    self.angular = 0.0
                    self.last_valid_time = 0.0
                self.get_logger().warning(f'Serial input error: {exc}')

    def publish_command(self):
        msg = Twist()

        with self.lock:
            fresh = (
                self.last_valid_time > 0.0
                and time.monotonic() - self.last_valid_time
                <= self.stale_timeout
            )

            if fresh:
                msg.linear.x = self.linear * self.max_linear
                msg.angular.z = self.angular * self.max_angular

        self.publisher.publish(msg)

    def destroy_node(self):
        if self.serial_port is not None:
            self.serial_port.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RadioCtrlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
