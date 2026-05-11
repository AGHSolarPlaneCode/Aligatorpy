import cv2
import glob
from DistortionCalibrator import DistortionCalibrator

# program przekształca zdjęcia z folderu 'images', po czym zapisuje je w folderze projektu
images = glob.glob('images/*.jpg')
i = 1
for f_name in images:
    img = cv2.imread(f_name)
    fixed_img = DistortionCalibrator.correct_distortion(img)
    cv2.imwrite(f'wynik_kalibracji{i}.jpg', fixed_img)
    i += 1
