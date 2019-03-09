#!/usr/bin/env python
# -*- coding:utf-8 -*-
__author__ = "Rishav Rajendra"
__license__ = "MIT"
__status__ = "Development"

import cv2
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
from constants import CAMERA_RESOLUTION, CAMERA_FRAMERATE

import sys
sys.path.append("../tensorflow_duckling/models/research/object_detection/")
from image_processing import Model

import warnings
warnings.filterwarnings('ignore')

def main():
    # Initialize frame rate calculation
    frame_rate_calc = 1
    freq = cv2.getTickFrequency()
    font = cv2.FONT_HERSHEY_SIMPLEX

    # Initialize camera and grab reference to the raw capture
    camera = PiCamera()
    camera.resolution = CAMERA_RESOLUTION
    camera.framerate = CAMERA_FRAMERATE
    rawCapture = PiRGBArray(camera, size=CAMERA_RESOLUTION)
    rawCapture.truncate(0)

    objectifier = Model()

    for frame1 in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        t1 = cv2.getTickCount()

        frame = np.copy(frame1.array)
        processed_frame = objectifier.predict(frame)

        cv2.putText(processed_frame, "FPS: {0:.2f}".format(frame_rate_calc),(30,50),font,1,(255,255,0),2,cv2.LINE_AA)

        cv2.imshow('Object detector', processed_frame)

        t2 = cv2.getTickCount()
        time1 = (t2-t1)/freq
        frame_rate_calc=1/time1

        if cv2.waitKey(1) == ord('q'):
            break

        rawCapture.truncate(0)
    camera.close()

if __name__ == '__main__':
    main()
