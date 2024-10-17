from threading import Thread

from loguru import logger

from lib.cameras import Camera, StereoCamera
from lib.skynet import SkyNetAPI

STEREO_CAMERA_ID = 2
STEREO_CAMERA_FRAME_WIDTH = 640
STEREO_CAMERA_FRAME_HEIGHT = 240

BOTTOM_CAMERA_ID = 0
BOTTOM_CAMERA_FRAME_WIDTH = 320
BOTTOM_CAMERA_FRAME_HEIGHT = 240

ARDUINO_MEGA_PORT = "/dev/ttyACM0"
ARDUINO_MEGA_BAUDRATE = 9600


class MEGA:
    def __init__(self) -> None:
        self.mode: str = "None"
        self.azimuth: int = 0

        self.flv: int = 54
        self.frv: int = 54
        self.blv: int = 54
        self.brv: int = 54
        self.cube: int = 0
        self.winch: int = 0

        self.vel_range: tuple[int, int] = (10, 98)

        # SkyNet
        self.sn = SkyNetAPI()

        self.sc = StereoCamera(
            id=STEREO_CAMERA_ID,
            width=STEREO_CAMERA_FRAME_WIDTH,
            height=STEREO_CAMERA_FRAME_HEIGHT,
        )

        self.bc = Camera(
            id=BOTTOM_CAMERA_ID,
            width=BOTTOM_CAMERA_FRAME_WIDTH,
            height=BOTTOM_CAMERA_FRAME_HEIGHT,
        )

        # self.thread = Thread(target=self.__tick)
        # self.thread.daemon = True
        # self.thread.start()

    @property
    def tx_pkg(self) -> str:
        return f"{int(self.flv)}{int(self.frv)}{int(self.blv)}{int(self.brv)}{int(self.cube)}{int(self.winch)}{0}$"

    def set_omni_vel(
        self,
        flv: int | float = 54,
        blv: int | float = 54,
        frv: int | float = 54,
        brv: int | float = 54,
    ) -> None:
        self.flv = int(min(self.vel_range[1], max(self.vel_range[0], flv)))
        self.blv = int(min(self.vel_range[1], max(self.vel_range[0], blv)))
        self.frv = int(min(self.vel_range[1], max(self.vel_range[0], frv)))
        self.brv = int(min(self.vel_range[1], max(self.vel_range[0], brv)))

    def add_omni_vel(
        self,
        flv: int | float = 54,
        blv: int | float = 54,
        frv: int | float = 54,
        brv: int | float = 54,
    ) -> None:
        self.flv = int(min(self.vel_range[1], max(self.vel_range[0], self.flv + flv)))
        self.blv = int(min(self.vel_range[1], max(self.vel_range[0], self.blv + blv)))
        self.frv = int(min(self.vel_range[1], max(self.vel_range[0], self.frv + frv)))
        self.brv = int(min(self.vel_range[1], max(self.vel_range[0], self.brv + brv)))

    def set_diff_vel(self, l_vel: int | float = 54, r_vel: int | float = 54) -> None:
        self.set_left_vel(int(l_vel))
        self.set_right_vel(int(r_vel))

    def set_left_vel(self, vel: int | float) -> None:
        self.flv = int(min(self.vel_range[1], max(self.vel_range[0], vel)))
        self.blv = int(min(self.vel_range[1], max(self.vel_range[0], vel)))

    def set_right_vel(self, vel: int) -> None:
        self.frv = int(min(self.vel_range[1], max(self.vel_range[0], vel)))
        self.brv = int(min(self.vel_range[1], max(self.vel_range[0], vel)))

    def set_cube_state(self, state: int = 0) -> None:
        self.cube = state

    def set_winch_state(self, state: int = 0) -> None:
        self.winch = state

    def idle(self) -> None:
        self.set_omni_vel()
        self.set_cube_state(0)
        self.set_winch_state(0)
