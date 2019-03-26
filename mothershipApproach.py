import cv2
import numpy as np
from nav.grid import Grid
from nav.gridMovement import GridMovement
from video_thread import VideoThread
from get_stats_from_image import get_angle, get_distance, get_data
from serial import Serial
from constants import fwd, rev, rotl, rotr, strl, strr, \
        CAMERA_RESOLUTION, CAMERA_FRAMERATE
import time, math, threading, queue

import sys
sys.path.append("../tensorflow_duckling/models/research/object_detection/")
from image_processing import Model

def main():
    objectifier = Model()

    # Initialize serial communication
    ser = Serial('/dev/ttyACM0',9600, timeout=2)
    time.sleep(1)

    # Initialize Grid movement for movement
    grid = Grid(8,8)
    movement = GridMovement(grid, ser)
    movement.move(fwd, 2)

if __name__ == '__main__':
    main()
