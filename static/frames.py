from cv2 import imread

from lib.vanischeCV import Frame

EMPTY_FRAME_320x240 = Frame(
    src=imread("static/empty-frame_320x240.png"), colorspace="bgr"
)
EMPTY_FRAME_640x480 = Frame(
    src=imread("static/empty-frame_640x480.png"), colorspace="bgr"
)
