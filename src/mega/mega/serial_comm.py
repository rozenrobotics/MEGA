# sudo chmod 666 /dev/ttyACM0

from time import time

import rclpy
from rclpy.node import Node
from serial import STOPBITS_ONE, Serial
from std_msgs.msg import Int32, String

from .submodules.FPS import FPS


class SerialComm(Node):
    def __init__(self):
        super().__init__("serial_comm")

        self.fps = FPS()

        try:
            self.serial = Serial(
                port="/dev/ttyACM0",
                baudrate=115200,
                stopbits=STOPBITS_ONE,
            )
        except Exception as ex:
            self.get_logger().fatal(f"Serial port /dev/ttyACM0 was not opened: {ex}")

        self.received = False
        self.terminator = "$"

        self.subs = {
            "output_pkg": self.create_subscription(
                String, "/mega/serial_comm/output_pkg", self.__communicate, 10
            )
        }

        self.pubs = {
            "fps": self.create_publisher(Int32, "/mega/serial_comm/fps", 10),
            "mode": self.create_publisher(String, "/mega/mode", 10),
            "azimuth": self.create_publisher(Int32, "/mega/azimuth", 10),
        }

    def __communicate(self, msg: String) -> None:
        self.fps.tick()

        _fps_msg = Int32()
        _fps_msg.data = self.fps.value

        self.pubs["fps"].publish(_fps_msg)

        if self.received:
            self.serial.write((msg.data + self.terminator).encode("utf-8"))

            self.received = False
            return

        _char = None
        _input_pkg = ""
        _dead_tmr = time()

        if not (self.serial.in_waiting > 0):
            return

        while True:
            try:
                _char = str(self.serial.read(), "utf-8")
            except Exception as ex:
                self.get_logger().warn(f"Could not Read Char from port: {ex}")
                break

            if _char != self.terminator:
                _input_pkg += _char

            else:
                self.input_pkg = _input_pkg

                if len(self.input_pkg) == 4:
                    _mode_msg = String()
                    _mode_msg.data = (
                        "manual" if bool(self.input_pkg[0]) else "autonomous"
                    )

                    _azimuth_msg = Int32()
                    _azimuth_msg.data = int(self.input_pkg[1:]) - 200

                    self.pubs["mode"].publish(_mode_msg)
                    self.pubs["azimuth"].publish(_azimuth_msg)
                break

            if _dead_tmr + 0.05 < time():
                self.get_logger().warn(f"Could not Read Package from port: TIMEOUT")
                break

        self.serial.reset_input_buffer()
        self.received = True


def main(args=None):
    rclpy.init(args=args)
    node = SerialComm()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
