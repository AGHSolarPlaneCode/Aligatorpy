import cv2
import numpy as np
import os

os.makedirs("porownanie", exist_ok=True)

K_fisheye = np.array([[
        420.56624654,
        0.0,
        313.86132572
    ],
    [
        0.0,
        421.58131545,
        203.36702095
    ],
    [
        0.0,
        0.0,
        1.0
    ]])

D_fisheye = np.array([[-0.06971922],
                      [ 0.07964969],
                      [-0.12121845],
                      [ 0.05742613]])

K_std = np.array([[484.75473137, 0.0, 317.03868198],
                  [0.0, 683.98693441, 236.66819013],
                  [0.0, 0.0, 1.0]])

D_std = np.array([[-0.10148394, 0.05831192, 0.05985018, 0.01358461, -0.60802582]])

img = cv2.imread('images/zdjecie_1.jpg')

if img is not None:
    h, w = img.shape[:2]

    new_K_fisheye = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(K_fisheye, D_fisheye, (w, h), np.eye(3), balance=1.0)
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K_fisheye, D_fisheye, np.eye(3), new_K_fisheye, (w, h), cv2.CV_16SC2)
    undistorted_fisheye = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

    new_K_std, roi = cv2.getOptimalNewCameraMatrix(K_std, D_std, (w, h), 1, (w, h))
    undistorted_std = cv2.undistort(img, K_std, D_std, None, new_K_std)

    cv2.imwrite("porownanie/1_oryginal.jpg", img)
    cv2.imwrite("porownanie/2_standard.jpg", undistorted_std)
    cv2.imwrite("porownanie/3_fisheye.jpg", undistorted_fisheye)

    cv2.imshow('Oryginal', img)
    cv2.imshow('Wyprostowane (Standard)', undistorted_std)
    cv2.imshow('Wyprostowane (Fisheye)', undistorted_fisheye)

    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print("Nie znaleziono pliku.")