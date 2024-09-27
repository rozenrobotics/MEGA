import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32

from .submodules.FPS import FPS


class BottomCamera(Node):
    def __init__(self) -> None:
        super().__init__("bottom_camera")

        self.cap = cv2.VideoCapture(0)
        self.cv_bridge_ = CvBridge()
        self.fps = FPS()

        self.capture_tmr = self.create_timer(1 / 30, self.__process)

        self.bgr = None
        self.hsv = None
        self.mon = None

        self.pubs = {
            "bgr": self.create_publisher(Image, "/mega/bottom_camera/bgr", 10),
            "hsv": self.create_publisher(Image, "/mega/bottom_camera/hsv", 10),
            "mon": self.create_publisher(Image, "/mega/bottom_camera/mon", 10),
            "fps": self.create_publisher(Int32, "/mega/bottom_camera/fps", 10),
        }

    def __process(self) -> None:
        self.fps.tick()

        _ret, _frame = self.cap.read()
        _fps_msg = Int32()
        _fps_msg.data = self.fps.value

        if not _ret:
            self.get_logger().info("Can not Capture the frame!")

        self.bgr = _frame
        self.hsv = cv2.cvtColor(self.bgr, cv2.COLOR_BGR2HSV)
        self.mon = cv2.cvtColor(self.bgr, cv2.COLOR_BGR2GRAY)

        self.pubs["bgr"].publish(self.cv_bridge_.cv2_to_imgmsg(self.bgr, "bgr8"))
        self.pubs["hsv"].publish(self.cv_bridge_.cv2_to_imgmsg(self.hsv, "bgr8"))
        self.pubs["mon"].publish(self.cv_bridge_.cv2_to_imgmsg(self.mon, "mono8"))
        self.pubs["fps"].publish(_fps_msg)


def main(args=None):
    rclpy.init(args=args)
    node = BottomCamera()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
