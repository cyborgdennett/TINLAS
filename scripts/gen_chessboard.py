import cv2
import sys
import numpy as np
import os
import tempfile

def checkerboard(shape, tile_size):
    return (np.indices(shape, dtype="uint8") // tile_size).sum(axis=0) % 2 * 255


def gen_chess(width):
    # check if file exists
    if os.path.exists(tempfile.gettempdir() + f"/chess{width}x{width}.jpg"):
        return
    
    
    im = checkerboard((width,width,1), width/2)

    f = tempfile.NamedTemporaryFile()
    f.name = tempfile.gettempdir() + f"/chess{width}x{width}.jpg"

    cv2.imwrite(f.name, im)
    
    # cv2.imshow('image', bg)
    # cv2.waitKey(0)
    
if __name__ == '__main__':
    gen_chess(256)