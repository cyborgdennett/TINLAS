import cv2
import sys
import numpy as np
import os
import tempfile

def checkerboard(shape, tile_size):
    return (np.indices(shape, dtype="uint8") // tile_size).sum(axis=0) % 2 * 255


def gen_chess(width, height, tile_size):
    # check if file exists
    if os.path.exists(tempfile.gettempdir() + f"/chess{width}x{width}.jpg"):
        return
    
    
    im = checkerboard((width,height,1), tile_size)

    f = tempfile.NamedTemporaryFile()
    f.name = tempfile.gettempdir() + f"/chess{width}x{width}.jpg"

    cv2.imwrite(f.name, im)
    
    # cv2.imshow('image', bg)
    # cv2.waitKey(0)
    
if __name__ == '__main__':
    gen_chess(1024+256,512+256+128,128)
    
# The A4 that this generated in portrait is:
    # gen_chess(1024+256,512++256+128,128)
    # x: 7 squares x:19.3cm, x:39.0cm d:19.7cm squareSize: 2.814
    # y: 10 squares y:19.9cm, y:48.4cm d:28.5cm  squareSize: 2.85