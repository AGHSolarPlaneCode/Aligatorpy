import cv2
import glob
import numpy as np

# program analizuje zdjęcia z folderu 'images', po czym wylicza macierz kamery oraz współczynnik dystorcji
# z tych zmiennych korzysta klasa DistortionCalibrator

# LICZBA STYKÓW KWADRATÓW W SZEROKOŚCI X WYSOKOŚCI
CHECKERBOARD = (19, 14)

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

all_obj_points = []
all_img_points = []

# ŚCIEŻKA ZDJĘĆ DO PRZEANALIZOWANIA
images = glob.glob('images/*.jpg')

for f_name in images:
    img = cv2.imread(f_name)

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # RODZAJ ANALIZOWANEJ SZACHOWNICY
    # W PRZYPADKU ZWYKŁEJ SZACHOWNICY (ZŁOŻONEJ Z SAMYCH KWADRATÓW) ZAMIENIĆ NA: cv2.findChessboardCorners(...)
    ret, corners = cv2.findChessboardCornersSB(gray, CHECKERBOARD,
        cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)

    if ret:
        print(f"Znaleziono wzór na: {f_name}")
        all_obj_points.append(objp)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        all_img_points.append(corners2)
    else:
        print(f"Nie wykryto szachownicy na: {f_name}")


ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(all_obj_points, all_img_points, gray.shape[::-1], None, None)


print("\n--- Wyniki kalibracji ---\n")
print("Macierz kamery (Intrinsic Matrix):")
print(mtx)
print("\nWspółczynniki dystorsji (Distortion Coefficients):")
print(dist)
