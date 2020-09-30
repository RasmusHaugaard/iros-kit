import cv2
import json
from iros_kit.pose_extraction.svg_to_img import svg_to_numpy

obj_config = json.load(open('../obj_config.json'))

img = svg_to_numpy('../../layouts/practice.svg')
templates = []

for name in obj_config.keys():
    roi = cv2.selectROI(name, img, showCrosshair=True)
    cv2.destroyWindow(name)
    if roi == (0, 0, 0, 0):
        break
    x, y, w, h = roi
    cv2.imwrite(f'../templates/{name}.png', img[y:y + h, x:x + w])
