import cv2 as cv
import rerun as rr 
import numpy as np

from submodules.vanischeCV import *

rr.init('stereo', spawn=True)

cap = cv.VideoCapture(2)
cap.set(cv.CAP_PROP_FPS, 30)
cap.set(cv.CAP_PROP_FRAME_WIDTH, 320*2)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 240)

stereo = cv.StereoBM_create(numDisparities=96, blockSize=25)

def split_stereo_frame(frame: Frame) -> tuple[Frame, Frame]:
    l_frame = frame.roi(
            ROI(
                0,
                frame.h,
                0,
                frame.w // 2,
                )
            )
    r_frame = frame.roi(
            ROI(
                0, 
                frame.h,
                frame.w // 2,
                frame.w
                )
            )

    return (l_frame, r_frame)

def compute_depth_map(gray_splitted: tuple[Frame, Frame]) -> Frame:
    disparity = stereo.compute(gray_splitted[0].src, gray_splitted[1].src) / 128
    
    return Frame(disparity, 'gray')


while cv.waitKey(1) != ord('q'):
    ret, raw = cap.read()

    if not ret:
        print('not ret')
        continue 

    raw_bgr = Frame(raw, 'bgr')
    raw_rgb = raw_bgr.cvt2rgb()

    raw_splitted:  tuple[Frame, Frame] = split_stereo_frame(raw_bgr)
    gray_splitted: tuple[Frame, Frame] = (raw_splitted[0].cvt2gray(), raw_splitted[1].cvt2gray())

    map: Frame = compute_depth_map(gray_splitted)

    adapt_map_mask = Frame(
        cv.adaptiveThreshold(
            map.src.astype('uint8'),
            255,
            cv.ADAPTIVE_THRESH_MEAN_C,
            cv.THRESH_BINARY_INV,
            21,
            15,
        ),
        'gray',
    )

    otsus_map_mask = Frame(
        cv.threshold(
            map.src.astype('uint8'),
            0,
            255,
            cv.THRESH_BINARY_INV | cv.THRESH_OTSU,
        )[1],
        'gray',
    )

    gauss_map_mask = Frame(
        cv.adaptiveThreshold(
            map.src.astype('uint8'),
            255,
            cv.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv.THRESH_BINARY_INV,
            5, 
            5,
        ),
        'gray',
    )

    thrsh_map_mask = Frame(
        cv.threshold(
            map.src.astype('uint8'),
            0.1,
            255,
            cv.THRESH_BINARY_INV,
        )[1],
        'gray',
    )

    thrsh = thrsh_map_mask.src.copy()

    thrsh_map_mask.src = cv.morphologyEx(
        thrsh_map_mask.src,
        cv.MORPH_CLOSE,
        np.ones((5, 5), np.uint8),
    )


    rr.log('map', rr.Image(map.src))
    rr.log('thrsh map mask', rr.Image(thrsh_map_mask.src))
    rr.log('thrsh', rr.Image(thrsh))
    rr.log('adapt map mask', rr.Image(adapt_map_mask.src))
    rr.log('otsus map mask', rr.Image(otsus_map_mask.src))
    rr.log('gauss map mask', rr.Image(gauss_map_mask.src))

