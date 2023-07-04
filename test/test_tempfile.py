import cv2
import sys
import numpy as np
import os
import tempfile


marker_id = 1

output = "aruco_{}".format(marker_id)
arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)

width = 256
_w = width - int(2 * width/9)
print(f"width: {width}, _w: {_w}")

bg = np.ones((width, width, 1), dtype="uint8")
bg.fill(255)

tag = np.ones((_w, _w, 1), dtype="uint8")
cv2.aruco.generateImageMarker(
    arucoDict, marker_id, _w, tag, 1
)

x_offset=y_offset = int(width/9)
# add white border
bg[y_offset:y_offset+tag.shape[0], x_offset:x_offset+tag.shape[1]] = tag


f = tempfile.NamedTemporaryFile()
f.name = tempfile.gettempdir() + f"/aruco_{marker_id}.jpg"

cv2.imwrite(f.name, bg)

img = cv2.imread(f.name)

cv2.imshow("img", img)
cv2.waitKey(0)