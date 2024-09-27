from os import walk

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32

from .submodules.FPS import FPS


class StereoCamera(Node):
    def __init__(self):
        super().__init__("stereo_camera")

        self.fps = FPS()
        self.cv_bridge_ = CvBridge()
        self.cap = cv2.VideoCapture(0)

        self.capture_tmr = self.create_timer(1 / 30, self.__process)

        self.stereo_frame = None
        self.left_bgr = None
        self.left_hsv = None
        self.left_mon = None
        self.right_bgr = None
        self.right_hsv = None
        self.right_mon = None

        self.pubs: dict = {
            "src_frame": self.create_publisher(
                Image, "/mega/stereo_camera/src_frame", 10
            ),
            "left/bgr": self.create_publisher(
                Image, "/mega/stereo_camera/left/bgr", 10
            ),
            "left/hsv": self.create_publisher(
                Image, "/mega/stereo_camera/left/hsv", 10
            ),
            "left/mon": self.create_publisher(
                Image, "/mega/stereo_camera/left/mono", 10
            ),
            "right/bgr": self.create_publisher(
                Image, "/mega/stereo_camera/right/bgr", 10
            ),
            "right/hsv": self.create_publisher(
                Image, "/mega/stereo_camera/right/hsv", 10
            ),
            "right/mon": self.create_publisher(
                Image, "/mega/stereo_camera/right/mono", 10
            ),
            "fps": self.create_publisher(Int32, "/mega/stereo_camera/fps", 10),
        }

    def __process(self) -> None:
        _ret, _frame = self.cap.read()

        if not _ret:
            self.get_logger().info("Can not Capture the Frame!")
            return

        self.stereo_frame = _frame

        self.left_bgr = self.stereo_frame[
            0 : _frame.shape[0],
            0 : _frame.shape[1] // 2,
        ]
        self.left_hsv = cv2.cvtColor(self.left_bgr, cv2.COLOR_BGR2HSV)
        self.left_mon = cv2.cvtColor(self.left_bgr, cv2.COLOR_BGR2GRAY)

        self.right_bgr = self.stereo_frame[
            0 : _frame.shape[0],
            _frame.shape[1] // 2 : _frame.shape[1],
        ]
        self.right_hsv = cv2.cvtColor(self.right_bgr, cv2.COLOR_BGR2HSV)
        self.right_mon = cv2.cvtColor(self.right_bgr, cv2.COLOR_BGR2GRAY)

        self.pubs["src_frame"].publish(
            self.cv_bridge_.cv2_to_imgmsg(self.stereo_frame, "bgr8")
        )
        self.pubs["left/bgr"].publish(
            self.cv_bridge_.cv2_to_imgmsg(self.left_bgr, "bgr8")
        )
        self.pubs["left/hsv"].publish(
            self.cv_bridge_.cv2_to_imgmsg(self.left_hsv, "bgr8")
        )
        self.pubs["left/mon"].publish(
            self.cv_bridge_.cv2_to_imgmsg(self.left_mon, "mono8")
        )
        self.pubs["right/bgr"].publish(
            self.cv_bridge_.cv2_to_imgmsg(self.right_bgr, "bgr8")
        )
        self.pubs["right/hsv"].publish(
            self.cv_bridge_.cv2_to_imgmsg(self.right_hsv, "bgr8")
        )
        self.pubs["right/mon"].publish(
            self.cv_bridge_.cv2_to_imgmsg(self.right_mon, "mono8")
        )

        self.fps.tick()
        _fps_msg = Int32()
        _fps_msg.data = self.fps.value

        self.pubs["fps"].publish(_fps_msg)


def main(args=None):
    rclpy.init(args=args)
    node = StereoCamera()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
