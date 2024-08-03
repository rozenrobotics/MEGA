from submodules.MEGA import MEGA
import cv2 as cv 


if __name__ == "__main__":
    cap = cv.VideoCapture(0)

    while cv.waitKey(1) != ord('q'):
        ret, frame = cap.read()

        if not ret:
            continue

    
