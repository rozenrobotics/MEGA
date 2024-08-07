import cv2
import pickle

# Load the calibration parameters
with open('stereo_calibration.pkl', 'rb') as f:
    calibration_data = pickle.load(f)

camera_matrix_left = calibration_data['camera_matrix_left']
dist_coeffs_left = calibration_data['dist_coeffs_left']
camera_matrix_right = calibration_data['camera_matrix_right']
dist_coeffs_right = calibration_data['dist_coeffs_right']
R = calibration_data['R']
T = calibration_data['T']

# Initialize the camera captures
cap = cv2.VideoCapture(0)

# Stereo rectify
R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(camera_matrix_left, dist_coeffs_left, camera_matrix_right, dist_coeffs_right, (640, 480), R, T)

# Create stereo block matching object
stereo = cv2.StereoSGBM_create(
    minDisparity=0,
    numDisparities=16*4,
    blockSize=5,
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
    # Capture frame-by-frame
    ret, frame = cap.read()

    if not ret:
        continue 

    frame_left = frame[0:240, 0:320]
    frame_right = frame[0:240, 320:640]

    # Rectify the images
    map1_left, map2_left = cv2.initUndistortRectifyMap(camera_matrix_left, dist_coeffs_left, R1, P1, (640, 480), cv2.CV_16SC2)
    map1_right, map2_right = cv2.initUndistortRectifyMap(camera_matrix_right, dist_coeffs_right, R2, P2, (640, 480), cv2.CV_16SC2)
    frame_left_rectified = cv2.remap(frame_left, map1_left, map2_left, cv2.INTER_LANCZOS4)
    frame_right_rectified = cv2.remap(frame_right, map1_right, map2_right, cv2.INTER_LANCZOS4)

    # Compute disparity map
    disparity = stereo.compute(frame_left_rectified, frame_right_rectified)

    # Normalize the disparity map for visualization
    disparity_normalized = cv2.normalize(disparity, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)


    cv2.imshow('Left Camera', cv2.hconcat([frame_left_rectified, frame_right_rectified]))
    cv2.imshow('Disparity Map', cv2.hconcat([disparity_normalized.astype('uint8'), disparity.astype('uint8')]))



    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything is done, release the capture
left_camera.release()
right_camera.release()
cv2.destroyAllWindows()
