from time import time

import rclpy
from rclpy.node import Node
from serial import STOPBITS_ONE, Serial


class SerialComm(Node):
    def __init__(self):
        super().__init__("serial")
        self.PORT = "/dev/ttyACM0"
        self.BAUDRATE = 115200
        self.TERMINATOR = "$"

        self.dead_tmr = time()
        self.receiver = False

        self.input_pkg = ""
        self.output_pkg = ""

    def connect(self) -> None:
        self.serial = Serial(
            port=self.PORT, baudrate=self.BAUDRATE, stopbits=STOPBITS_ONE
        )

        if self.serial.is_open:
            self.get_logger().info(f"Serial {self.PORT} Connected!")
            return

        self.get_logger().info(f"Serial {self.PORT} is NOT Connected!")


def main(args=None):
    rclpy.init(args=args)
    node = SerialComm()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
