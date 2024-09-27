from time import time

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, String

from .submodules.FPS import FPS


class StateMachine(Node):
    def gates(self) -> None:
        if self.state_tmr + 1 < time():
            self.update_state("marker")

    def marker(self) -> None:
        if self.state_tmr + 1 < time():
            self.update_state("finish")

    def finish(self) -> None:
        if self.state_tmr + 1 < time():
            self.update_state("gates")

    def idle(self) -> None:
        self.update_state("gates")

    def __tick(self) -> None:
        self.fps.tick()

        _fps_msg = Int32()
        _fps_msg.data = self.fps.value

        self.pubs["fps"].publish(_fps_msg)

        match self.state:
            case "idle":
                self.idle()
            case "gates":
                self.gates()
            case "marker":
                self.marker()
            case "finish":
                self.finish()

        _serial_output_msg = String()
        _serial_output_msg.data = "44444444010"
        self.pubs["serial_output_pkg"].publish(_serial_output_msg)

    def __init__(self) -> None:
        super().__init__("state_machine")

        self.fps = FPS()
        self.cv_bridge_ = CvBridge()

        self.state = "idle"
        self.last_state = None
        self.state_tmr = time()
        self.new_frames = False

        self.sc_src = None
        self.scl_bgr = None
        self.scl_hsv = None
        self.scl_mon = None
        self.scr_bgr = None
        self.scr_hsv = None
        self.scr_mon = None
        self.sc_depth_map = None

        self.mode = None
        self.azimuth = None
        self.serial_output_pkg = {
            "flm_vel": 100,
            "frm_vel": 100,
            "blm_vel": 100,
            "brm_vel": 100,
            "flajok0": 0,
            "flajok1": 1,
            "flajok2": 0,
        }

        self.pubs = {
            "fps": self.create_publisher(Int32, "/mega/state_machine/fps", 10),
            "state": self.create_publisher(String, "/mega/state_machine/state", 10),
            "serial_output_pkg": self.create_publisher(
                String, "/mega/serial_comm/output_pkg", 10
            ),
        }

        self.subs = {
            "/mega/azimuth": self.create_subscription(
                Int32,
                "/mega/azimuth",
                self.__set_azimuth,
                10,
            ),
            "/mega/mode": self.create_subscription(
                String,
                "/mega/mode",
                self.__set_mode,
                10,
            ),
            "/mega/stereo_camera/src_frame": self.create_subscription(
                Image,
                "mega/stereo_camera/src_frame",
                lambda msg: self.__bridge_frame(msg, "bgr8"),
                10,
            ),
            "/mega/stereo_camera/left/bgr": self.create_subscription(
                Image,
                "mega/stereo_camera/left/bgr",
                lambda msg: self.__bridge_frame(msg, "bgr8"),
                10,
            ),
            "/mega/stereo_camera/left/hsv": self.create_subscription(
                Image,
                "mega/stereo_camera/left/hsv",
                lambda msg: self.__bridge_frame(msg, "bgr8"),
                10,
            ),
            "/mega/stereo_camera/left/mon": self.create_subscription(
                Image,
                "mega/stereo_camera/left/mon",
                lambda msg: self.__bridge_frame(msg, "mono8"),
                10,
            ),
            "/mega/stereo_camera/right/bgr": self.create_subscription(
                Image,
                "mega/stereo_camera/right/bgr",
                lambda msg: self.__bridge_frame(msg, "bgr8"),
                10,
            ),
            "/mega/stereo_camera/right/hsv": self.create_subscription(
                Image,
                "mega/stereo_camera/right/hsv",
                lambda msg: self.__bridge_frame(msg, "bgr8"),
                10,
            ),
            "/mega/stereo_camera/right/mon": self.create_subscription(
                Image,
                "mega/stereo_camera/right/mon",
                lambda msg: self.__bridge_frame(msg, "mono8"),
                10,
            ),
            "/mega/stereo_camera/depth_map": self.create_subscription(
                Image,
                "/mega/stereo_camera/depth_map",
                lambda msg: self.__bridge_frame(msg, "mono8"),
                10,
            ),
        }

        self.tick_tmr = self.create_timer(1 / 30, self.__tick)

    def __bridge_frame(self, msg: Image, encoding: str) -> None:
        _frame = self.cv_bridge_.imgmsg_to_cv2(msg, encoding)

        match msg.header.frame_id:
            case "/mega/stereo_camera/src_frame":
                self.sc_src = _frame
            case "/mega/stereo_camera/left/bgr":
                self.scl_bgr = _frame
            case "/mega/stereo_camera/left/hsv":
                self.scl_hsv = _frame
            case "/mega/stereo_camera/left/hsv":
                self.scl_mon = _frame
            case "/mega/stereo_camera/left/hsv":
                self.scr_bgr = _frame
            case "/mega/stereo_camera/left/hsv":
                self.scr_hsv = _frame
            case "/mega/stereo_camera/left/hsv":
                self.scr_mon = _frame
            case "/mega/stereo_camera/depth_map":
                self.sc_depth_map = _frame

        self.new_frames = True

    def __set_azimuth(self, msg: Int32) -> None:
        self.azimuth = msg.data

    def __set_mode(self, msg: String) -> None:
        self.mode = msg.data

    def update_state(self, new_state: str = "idle") -> None:
        self.last_state = self.state
        self.state = new_state
        self.state_tmr = time()

        _state_msg = String()
        _state_msg.data = self.state
        self.get_logger().info(f"Current State -> {self.state}")

        self.pubs["state"].publish(_state_msg)


def main(args=None):
    rclpy.init(args=args)
    node = StateMachine()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
