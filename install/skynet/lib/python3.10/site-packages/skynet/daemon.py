import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32

from .submodules.FPS import FPS
from .submodules.SkyNetAPI import SkyNetAPI


class Daemon(Node):
    def __init__(self) -> None:
        super().__init__()

        self.fps = FPS()
        self.skynet = SkyNetAPI()
        self.cv_bridge_ = CvBridge()

        self.key = None

        self.subs = {
            "frame": self.create_subscription(
                Image, "/skynet/frame_to_host", self.__set_frame, 10
            )
        }

        self.pubs = {
            "fps": self.create_publisher(Int32, "/skynet/fps", 10),
            "key": self.create_publisher(Int32, "/skynet/key", 10),
        }

        self.key_tmr = self.create_tmr(1 / 30, self.__pub_key)

    def __set_frame(self, msg: Image) -> None:
        self.fps.tick()
        _fps_msg = Int32()
        _fps_msg.data = self.fps.value
        self.pubs["fps"].publish(_fps_msg)

        _cv2_frame = self.cv_bridge_.imgmsg_to_cv2(msg)
        self.skynet.set_frame(_cv2_frame)

    def __pub_key(self) -> None:
        _key_msg = Int32()
        _key_msg.data = self.skynet.get_key()

        self.pubs["key"].publish(_key_msg)
