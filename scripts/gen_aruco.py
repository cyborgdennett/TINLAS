
import argparse
import cv2
import sys
import numpy as np
import os

ap = argparse.ArgumentParser()
ap.add_argument("-o", "--output", help="path to output image containing ArUCo tag")
ap.add_argument("-i", "--id", type=int, required=True, help="ID of first ArUCo tag to generate")
ap.add_argument("-t", "--type", type=str, default="DICT_ARUCO_ORIGINAL", help="type of ArUCo tag to generate")
ap.add_argument("-w", "--width", type=str, default="256", help="pixels per side of the ArUCo tag")
ap.add_argument("-s", "--size", type=int, default=50, help="the size in mm of the ArUco tag")
ap.add_argument("-m", "--margin", type=int, default=5, help="the size in mm of the margins between the ArUco tags")
ap.add_argument("--write-id", default=True, action=argparse.BooleanOptionalAction, help="write the id of the tag or not")
args = vars(ap.parse_args())

ARUCO_DICT = {
	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
	"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
	"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
	"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
	"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}

if ARUCO_DICT.get(args["type"], None) is None:
	print("[INFO] ArUCo tag of '{}' is not supported".format(args["type"]))
	sys.exit(0)
 
if args["output"] is None:
    args["output"] = "aruco_{}".format(str(args["id"]))

arucoDict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[args["type"]])

width = int(args["width"])
_w = width - int(2 * width/9)
print(f"width: {width}, _w: {_w}")

bg = np.ones((width, width, 1), dtype="uint8")
bg.fill(255)

tag = np.ones((_w, _w, 1), dtype="uint8")
cv2.aruco.generateImageMarker(
    arucoDict, int(args["id"]), _w, tag, 1
)

x_offset=y_offset = int(width/9)
bg[y_offset:y_offset+tag.shape[0], x_offset:x_offset+tag.shape[1]] = tag

dirpath = os.getcwd()
file_name = os.path.join(dirpath, args["output"] + ".jpg")
cv2.imwrite(file_name, bg)
