import cv2
import numpy as np

#
CAM_MTX = np.array([[8.68481984e+03, 0.00000000e+00, 1.88383102e+03],
                    [0.00000000e+00, 8.18780065e+03, 9.58295477e+02],
                    [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]], dtype=np.float32)
DIST_COEF = np.array([[0.88588838, -6.50463242, -0.02164041, -0.02205379, 20.56777975]], dtype=np.float32)


class DistortionCalibrator(object):
    @staticmethod
    def correct_distortion(img: np.ndarray) -> np.ndarray:
        h, w = img.shape[:2]

        new_camera_mtx, roi = cv2.getOptimalNewCameraMatrix(CAM_MTX, DIST_COEF, (w, h), 1, (w, h))

        fixed_img = cv2.undistort(img, CAM_MTX, DIST_COEF, None, new_camera_mtx)

        x, y, w_roi, h_roi = roi
        dst = fixed_img[y:y + h_roi, x:x + w_roi]

        return fixed_img
