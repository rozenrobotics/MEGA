import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.mgs import Image
from std_msgs.msg import Int32

from .submodules.FPS import FPS


class DepthMapComputer(Node):
    def __init__(self) -> None:
        super().__init__("depth_map_computer")

        self.fps = FPS()
        self.cv_bridge_ = CvBridge()

        self.scl_mon = None
        self.scr_mon = None

        self.compute_tmr = self.create_timer(1 / 30, self.__compute)
        self.depth_map_pub = self.create_publisher(Image, "/mega/depth_map", 10)

        self.scl_mon_sub = self.create_subscription(
            Image, "/mega/stereo_camera/left/mon", self.__bridge_frame
        )
        self.scr_mon_sub = self.create_subscription(
            Image, "/mega/stereo_camera/right/mon", self.__bridge_frame
        )

    def __compute(self) -> None: ...

    def __bridge_frame(self, msg) -> None:
        self.fps.tick()

        _frame = self.cv_bridge_.imgmsg_to_cv2(msg)
        _fps_msg = Int32()
        _fps_msg.data = self.fps.value

        match msg.header.frame_id:
            case "/mega/stereo_camera/left/mon":
                self.scl_mon = _frame
            case "/mega/stereo_camera/right/mon":
                self.scr_mon = _frame


def main(args=None):
    rclpy.init(args=args)
    node = DepthMapComputer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
