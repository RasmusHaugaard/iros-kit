import cv2
import json
import svg_to_img

objects = json.load(open('../objects.json'))
objects = {key: val for key, val in objects.items() if 'up_kit' in val}

img = svg_to_img.svg_to_numpy('../IROS2020_KitLayout practice.svg')
templates = []

for name, _ in objects.items():
    roi = cv2.selectROI(name, img, showCrosshair=True)
    cv2.destroyWindow(name)
    if roi == (0, 0, 0, 0):
        break
    x, y, w, h = roi
    cv2.imwrite(f'../templates/{name}.png', img[y:y + h, x:x + w])
