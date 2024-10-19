import cv2 as cv 

from lib.skynet import SkyNetAPI 
from lib.MEGA import MEGA
from lib.video_recorder import VideoRecorder 


VIDEO_ID = 0

if __name__ == "__main__":
    mega = MEGA()
    skynet = SkyNetAPI()
    video_recorder = VideoRecorder(f"videos/video{VIDEO_ID}", frame_size=(640, 240))

    while True:
        if not mega.sc.new_frame or not mega.bc.new_frame:
            continue 

        frame = cv.hconcat([mega.sc.l, mega.bc.src])
        skynet.set_frame(
            frame,
            telemetry=[
            ],
        )
    
