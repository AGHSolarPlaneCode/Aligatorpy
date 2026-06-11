import cv2
import glob
import numpy as np

CHECKERBOARD = (9, 6)

# Moduł fisheye wymaga dodatkowego wymiaru w tablicy (kształt: 1, N, 3)
objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[0, :, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

all_obj_points = []
all_img_points = []

images = glob.glob('images/*.jpg')
gray_shape = None

for f_name in images:
    img = cv2.imread(f_name)
    if img is None:
        continue

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray_shape = gray.shape[::-1]

    ret, corners = cv2.findChessboardCornersSB(gray, CHECKERBOARD, 
                                               cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_NORMALIZE_IMAGE)

    if ret:
        print(f"Znaleziono wzór na: {f_name}")
        all_obj_points.append(objp)
        
        # Otrzymane punkty też muszą mieć dodatkowy wymiar (kształt: 1, N, 2)
        all_img_points.append(corners.reshape(1, -1, 2))
    else:
        print(f"Nie wykryto szachownicy na: {f_name}")

if all_obj_points:
    # Przygotowanie pustych macierzy dla wyników
    K = np.zeros((3, 3))
    D = np.zeros((4, 1))

    # Wymagane flagi, aby matematyka kalibracji fisheye była stabilna
    fisheye_flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_CHECK_COND | cv2.fisheye.CALIB_FIX_SKEW

    # Funkcja kalibrująca sferycznie
    rms, K, D, rvecs, tvecs = cv2.fisheye.calibrate(
        all_obj_points,
        all_img_points,
        gray_shape,
        K,
        D,
        rvecs=None,
        tvecs=None,
        flags=fisheye_flags,
        criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
    )

    print("\n--- Wyniki kalibracji Fisheye ---\n")
    print("Macierz kamery (Intrinsic Matrix K):")
    print(K)
    print("\nWspółczynniki dystorsji Fisheye (D):")
    print(D)
else:
    print("Brak udanych detekcji do kalibracji.")