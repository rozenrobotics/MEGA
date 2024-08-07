import cv2
import numpy as np


class Stereo:
    def __init__(self, cap_num: int) -> None:
        self.cap = cv2.VideoCapture(cap_num)

        self.frame = None
        self.l_raw = None
        self.r_raw = None

    def get_frame(self):
        ret, frame = self.cap.read()

        self.frame = frame
        self.l_raw = frame[0:240, 0:320]
        self.r_raw = frame[0:240, 0:640]

        return self.frame

            


stereo = Stereo(0)

while cv2.waitKey(1) != ord('q'):
    cv2.imshow('frame', stereo.get_frame())

