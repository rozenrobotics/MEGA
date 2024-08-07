import cv2

cap = cv2.VideoCapture(0)

# Create stereo block matching object
stereo = cv2.StereoSGBM_create(
    minDisparity=0,
    numDisparities=16*8,
    blockSize=15,
    P1=8 * 3 * 5 ** 2,
    P2=32 * 3 * 5 ** 2,
    disp12MaxDiff=1,
    uniquenessRatio=15,
    speckleWindowSize=100,
    speckleRange=2,
    preFilterCap=63,
    mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
)

while True:
    frame = cap.read()[1]
    frame_left, frame_right = frame[0:240, 0:320], frame[0:240, 320:640]

    # Convert to grayscale
    gray_left = cv2.cvtColor(frame_left, cv2.COLOR_BGR2GRAY)
    gray_right = cv2.cvtColor(frame_right, cv2.COLOR_BGR2GRAY)

    # Compute disparity map
    disparity = stereo.compute(gray_left, gray_right)

    # Normalize the disparity map for visualization
    disparity_normalized = cv2.normalize(disparity, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)

    # Display the resulting frames
    cv2.imshow('Disparity Map', cv2.hconcat([frame_left, cv2.cvtColor(disparity_normalized.astype('uint8'), cv2.COLOR_GRAY2BGR), frame_right]))

    # Press 'q' on the keyboard to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything is done, release the capture
left_camera.release()
right_camera.release()
cv2.destroyAllWindows()
