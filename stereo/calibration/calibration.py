import cv2
import numpy as np
import pickle

chessboard_size = (9, 6)

# Prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((np.prod(chessboard_size), 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)

# Arrays to store object points and image points from all the images.
objpoints = []  # 3d point in real world space
imgpoints_left = []  # 2d points in image plane.
imgpoints_right = []  # 2d points in image plane.

cap = cv2.VideoCapture(0)
cnt = 0

def calibrate_stereo_camera(objpoints, imgpoints_left, imgpoints_right, img_shape):
    # Calibrate the cameras
    ret_left, camera_matrix_left, dist_coeffs_left, _, _ = cv2.calibrateCamera(objpoints, imgpoints_left, img_shape, None, None)
    ret_right, camera_matrix_right, dist_coeffs_right, _, _ = cv2.calibrateCamera(objpoints, imgpoints_right, img_shape, None, None)

    # Stereo calibration
    retval, camera_matrix_left, dist_coeffs_left, camera_matrix_right, dist_coeffs_right, R, T, E, F = cv2.stereoCalibrate(
        objpoints, imgpoints_left, imgpoints_right, camera_matrix_left, dist_coeffs_left, camera_matrix_right, dist_coeffs_right, img_shape
    )

    return camera_matrix_left, dist_coeffs_left, camera_matrix_right, dist_coeffs_right, R, T

while True:
    ret, frame = cap.read()

    if not ret:
        continue

    frame_left, frame_right = frame[0:240, 0:320], frame[0:240, 320:640]

    gray_left = cv2.cvtColor(frame_left, cv2.COLOR_BGR2GRAY)
    gray_right = cv2.cvtColor(frame_right, cv2.COLOR_BGR2GRAY)

    ret_left, corners_left = cv2.findChessboardCorners(gray_left, chessboard_size, None)
    ret_right, corners_right = cv2.findChessboardCorners(gray_right, chessboard_size, None)

    if ret_left:
        cv2.drawChessboardCorners(frame_left, chessboard_size, corners_left, ret_left)

    if ret_right:
        cv2.drawChessboardCorners(frame_right, chessboard_size, corners_right, ret_right)


    # Display the resulting frames
    cv2.imshow('frame', cv2.hconcat([frame_left, frame_right]))

    if cv2.waitKey(1) & 0xFF == ord('n'):
        if ret_left and ret_right:
            cnt += 1
            objpoints.append(objp)
            imgpoints_left.append(corners_left)
            imgpoints_right.append(corners_right)
            print(f'| {cnt} | : ', "Chessboard detected and added for calibration.")
        else:
            print("Chessboard not detected in one or both images. Please try again.")

    if cv2.waitKey(1) & 0xFF == ord('c'):
        if len(objpoints) > 0:
            img_shape = gray_left.shape[::-1]
            camera_matrix_left, dist_coeffs_left, camera_matrix_right, dist_coeffs_right, R, T = calibrate_stereo_camera(objpoints, imgpoints_left, imgpoints_right, img_shape)
            
            # Save the calibration parameters
            calibration_data = {
                'camera_matrix_left': camera_matrix_left,
                'dist_coeffs_left': dist_coeffs_left,
                'camera_matrix_right': camera_matrix_right,
                'dist_coeffs_right': dist_coeffs_right,
                'R': R,
                'T': T
            }

            with open('stereo_calibration.pkl', 'wb') as f:
                pickle.dump(calibration_data, f)

            print("Stereo Calibration done and parameters saved.")
            break
        else:
            print("Not enough chessboard images for calibration. Please capture more images.")

    # Press 'q' on the keyboard to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything is done, release the capture
left_camera.release()
right_camera.release()
cv2.destroyAllWindows()
