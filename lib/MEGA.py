from threading import Thread

import cv2 as cv
from loguru import logger

from lib.BottomCamera import BottomCamera
from lib.SerialComm import SerialComm
from lib.SkyNetAPI import SkyNetAPI
from lib.StereoCamera import StereoCamera
from lib.vanischeCV import *


class MEGA:
    def __init__(self) -> None:
        logger.info("Init MEGA...")

        self.skynet = SkyNetAPI()

        self.stereo_cam = StereoCamera(cap_id=0)
        self.stereo_cam.open()

        self.bottom_cam = BottomCamera(cap_id=0)
        self.bottom_cam.open()
