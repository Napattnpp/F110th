import cv2
import numpy as np
from skimage.morphology import skeletonize

img_path = '/home/jetson/F110th/assets/maps/track715/track715.png'
raw_img = cv2.imread(img_path, 0)
if raw_img is None:
    print("Map not found")
    exit(1)

filtered_map = np.where(raw_img >= 250, 255, 0).astype(np.uint8)
filtered_map = cv2.flip(filtered_map, 0)
skeleton = skeletonize(filtered_map // 255, method='lee')

contours, hierarchy = cv2.findContours(skeleton.astype(np.uint8), cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
closed_contours = []
if hierarchy is not None:
    for i, elem in enumerate(contours):
        opened = hierarchy[0][i][2] < 0 and hierarchy[0][i][3] < 0
        if not opened:
            closed_contours.append(elem)

print(f"Total contours: {len(contours)}, closed contours: {len(closed_contours)}")

for i, cont in enumerate(closed_contours):
    line_length = 0
    for k, pnt in enumerate(cont):
        line_length += np.sqrt((pnt[0][0] - cont[k - 1][0][0]) ** 2 +
                               (pnt[0][1] - cont[k - 1][0][1]) ** 2)
    print(f"Closed Contour {i} pixel length: {line_length}")
    # multiply by resolution to get meters (assume 0.05 from cartographer)
    print(f"   -> meters (if res=0.05): {line_length * 0.05}")
