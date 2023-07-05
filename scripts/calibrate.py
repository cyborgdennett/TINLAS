""" Kudos to: https://aliyasineser.medium.com/opencv-camera-calibration-e9a48bdd1844 """
# import the necessary packages
from imutils.video import VideoStream
import imutils
import time
import cv2
import msvcrt
import numpy as np
import cv2

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


def calibrate(square_size, width=9, height=6):
    """ Apply camera calibration operation for images in the given directory path. """
    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(8,6,0)
    objp = np.zeros((height*width, 3), np.float32)
    objp[:, :2] = np.mgrid[0:width, 0:height].T.reshape(-1, 2)

    objp = objp * square_size

    # Arrays to store object points and image points from all the images.
    objpoints = []  # 3d point in real world space
    imgpoints = []  # 2d points in image plane.
    
    vs = VideoStream(src=0).start()
    time.sleep(2.0)
    
    while True:
        if msvcrt.kbhit():
            val = msvcrt.getch()
            print ("you pressed " + str(val) + " key")
            if val == b'q':
                print("quit")	
                break
            
            
        img = vs.read()
        img = imutils.resize(img, width=1000)

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        cv2.imshow('img', gray)
        cv2.waitKey(1) & 0xFF

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (width, height), None)

        # If found, add object points, image points (after refining them)
        if ret:
            print("found corners")	
            objpoints.append(objp)

            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)

            # Draw and display the corners
            img = cv2.drawChessboardCorners(img, (width, height), corners2, ret)
            
            cv2.imshow('img', img)
            cv2.waitKey(1000) & 0xFF
            
            time.sleep(1.0)

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    return [ret, mtx, dist, rvecs, tvecs]
def save_coefficients(mtx, dist, path):
    """ Save the camera matrix and the distortion coefficients to given path/file. """
    cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_WRITE)
    cv_file.write("K", mtx)
    cv_file.write("D", dist)
    # note you *release* you don't close() a FileStorage object
    cv_file.release()
def load_coefficients(path):
    """ Loads camera matrix and distortion coefficients. """
    # FILE_STORAGE_READ
    cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)

    # note we also have to specify the type to retrieve other wise we only get a
    # FileNode object back instead of a matrix
    camera_matrix = cv_file.getNode("K").mat()
    dist_matrix = cv_file.getNode("D").mat()

    cv_file.release()
    return [camera_matrix, dist_matrix]


if __name__ == '__main__':
    """" The A4 that generated the image is:
        gen_chess(1024+256,512++256+128,128)
    measurements with ruler:
     x: 7 squares x:19.3cm, x:39.0cm d:19.7cm squareSize: 2.814
     y: 10 squares y:19.9cm, y:48.4cm d:28.5cm  squareSize: 2.85 """
     
     """The downloaded img from the internet is:
     measurements with ruler:
     x: 20-36.5 d: 16.5cm squareSize: 1.5
     y: 30-51.5 d: 21.5cm squareSize: 2.55"""
    square_size = 2.85
    width = 6
    height = 9
    fileName = "calibration.yaml"
    
    ret, mtx, dist, rvecs, tvecs = calibrate(square_size, width, height)
    save_coefficients(mtx, dist, fileName)
    print("Calibration is finished. RMS: ", ret)