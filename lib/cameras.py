from threading import Thread

from cv2 import CAP_PROP_FRAME_HEIGHT, CAP_PROP_FRAME_WIDTH, VideoCapture, blur
from cv2.typing import MatLike
from loguru import logger


class Camera:
    def __init__(
        self,
        id: int = 0,
        width: int = 320,
        height: int = 240,
    ) -> None:
        self.id = id
        self.width = width
        self.height = height

        self.cap = VideoCapture(self.id)
        self.cap.set(CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(CAP_PROP_FRAME_HEIGHT, self.height)

        self._src: MatLike
        self._new_frame: bool = False

        self.thread = Thread(target=self.__tick)
        self.thread.daemon = True
        self.thread.start()

    @property
    def src(self) -> MatLike:
        return self._src

    @property
    def new_frame(self) -> bool:
        __new_frame = self._new_frame
        self._new_frame = False
        return __new_frame

    def __tick(self) -> None:
        while True:
            ret, frame = self.cap.read()

            if not ret:
                continue

            self._src = frame
            self._new_frame = True


class StereoCamera(Camera):
    def __init__(
        self,
        id: int = 0,
        width: int = 640,
        height: int = 240,
    ) -> None:
        super().__init__(id=id, width=width, height=height)

        # ROI: tuple[y0:y1, x0:x1]
        self.l_roi = [0, self.height, 0, self.width // 2]
        self.r_roi = [0, self.height, self.width // 2, self.width]

        self.BLUR_KERNEL = (5, 5)

    @property
    def l(self) -> MatLike:
        return blur(
                self.src[
                    self.l_roi[0] : self.l_roi[1],
                    self.l_roi[2] : self.l_roi[3],
                ],
                self.BLUR_KERNEL,
        )

    @property
    def r(self) -> MatLike:
        return blur(
                self.src[
                    self.r_roi[0] : self.r_roi[1],
                    self.r_roi[2] : self.r_roi[3],
                ],
                self.BLUR_KERNEL,
        )
