from threading import Thread

import cv2 as cv
from loguru import logger

from lib.FPS import FPS
from lib.vanischeCV import *
from static.frames import EMPTY_FRAME_320x240


class BottomCamera:
    def __init__(self, cap_id: int) -> None:
        self.cap_id = cap_id

        self.capture_thread = Thread(target=self.__get_frames)
        self.capture_thread.daemon = True

        self.raw_frame = EMPTY_FRAME_320x240

        self.fps = FPS()

    def open(self) -> None:
        self.cap = cv.VideoCapture(self.cap_id)

        if self.cap.isOpened():
            logger.success("Bottom Camera Opened Successfully!")
            self.capture_thread.start()
            logger.success("Capture Thread Started Successfully!")
            return
        logger.error("Bottom Camera is NOT Opened!")

    def __get_frames(self) -> None:
        while True:
            _ret, _frame = self.cap.read()

            if not _ret:
                logger.error("Could not capture the Frame")
                return

            self.raw_frame = Frame(src=_frame, colorspace="bgr")
            self.raw_frame_processed = False
            self.fps.tick()
