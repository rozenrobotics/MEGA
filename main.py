from time import time

import cv2 as cv
from loguru import logger

from lib.MEGA import MEGA
from lib.vanischeCV import *
from static.colors import *


class StateMachine:
    def __init__(self, mega: MEGA) -> None:
        self.mega = mega

        self.state = "init"
        self.tmr = time()

    def gates(self) -> None:
        if self.mega.stereo_cam.raw_frame_processed:
            return
        self.mega.stereo_cam.raw_frame_processed = True

    def marker(self) -> None: ...

    def tick(self) -> None:
        match self.state:
            case "init":
                logger.info("Init State Machine...")
                self.state = "gates"
            case "gates":
                self.gates()
            case "marker":
                self.marker()


if __name__ == "__main__":
    mega = MEGA()
    sm = StateMachine(mega)

    while cv.waitKey(1) != ord("q"):
        sm.tick()
