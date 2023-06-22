import cv2
from cv2 import aruco

markerImage = None
# TODO why it not work?
cv2.aruco.drawMarker(cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250), 0, [256,256], markerImage, 1)

cv2.imshow("frame", markerImage)
key = cv2.waitKey(1) & 0xFF