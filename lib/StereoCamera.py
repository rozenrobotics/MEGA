from threading import Thread

import cv2 as cv
from loguru import logger

from lib.FPS import FPS
from lib.vanischeCV import *
from static.frames import EMPTY_FRAME_320x240


class StereoCamera:
    def __init__(self, cap_id: int) -> None:
        self.cap_id = cap_id

        self.capture_thread = Thread(target=self.__get_frames)
        self.capture_thread.daemon = True

        self.raw_frame = EMPTY_FRAME_320x240
        self.left_frame = EMPTY_FRAME_320x240
        self.right_frame = EMPTY_FRAME_320x240

        self.raw_frame_processed = False
        self.left_frame_processed = False
        self.right_frame_processed = False

        self.fps = FPS()

    def open(self) -> None:
        self.cap = cv.VideoCapture(self.cap_id)

        if self.cap.isOpened():
            logger.success("Stereo Camera Opened Successfully!")
            self.capture_thread.start()
            logger.success("Capture Thread Started Successfully!")
            return
        logger.error("Stereo Camera is NOT Opened!")

    def __get_frames(self) -> None:
        while True:
            _ret, _raw_frame = self.cap.read()

            if not _ret:
                logger.error("Could not capture the Frame")
                return

            self.raw_frame = Frame(src=_raw_frame, colorspace="bgr")

            self.left_frame: Frame = self.raw_frame.roi(
                ROI(0, self.raw_frame.h, 0, self.raw_frame.w // 2)
            )
            self.right_frame: Frame = self.raw_frame.roi(
                ROI(0, self.raw_frame.h, self.raw_frame.w // 2, self.raw_frame.w)
            )

            self.raw_frame_processed = False
            self.left_frame_processed = False
            self.right_frame_processed = False

            self.fps.tick()
