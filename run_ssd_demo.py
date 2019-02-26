#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = "Rishav Rajendra"
__license__ = "MIT"
__status__ = "Development"

from vision.mobilenetv2_ssd_lite import create_mobilenetv2_ssd_lite, create_mobilenetv2_ssd_lite_predictor
from vision.utils.misc import Timer
from constants import CENTER_LINEx1y1, CENTER_LINEx2y2
import math
import cv2
import sys

import warnings
warnings.filterwarnings('ignore')
    
"""
If an obstacle is detected, draw a line from CENTER_LINEx2y2
to the center of the detected object
Calculate angle from the center line to align the robot with the obstacle
angle = arctan(m2-m1/(1+m1m2))
:param image: image to draw lines on
:param x1: Object x1 coordinate
:param y1: Object y1 coordinate
:param x2: Object x2 coordinate
:param y2: Object y2 coordinate
:return angle: (int)
"""
def detectObjects(image, x1, y1, x2, y2):
    center = ((x1 + x2)/2, (y1 + y2)/2)
    cv2.line(image, center, CENTER_LINEx2y2, (0, 0, 0))
    m1 = (CENTER_LINEx1y1[1] - CENTER_LINEx2y2[1])/((CENTER_LINEx1y1[0] - CENTER_LINEx2y2[0])+0.001)
    m2 = (center[1] - CENTER_LINEx2y2[1])/(center[0] - CENTER_LINEx2y2[0])
    div = (m2 - m1) / (1 + (m1 * m2))
    return int(math.degrees(math.atan(div)))

if len(sys.argv) < 5:
    print('Usage: python run_ssd_example.py <net type> <model path> <label path> <image path>')
    sys.exit(0)
net_type = sys.argv[1]
model_path = sys.argv[2]
label_path = sys.argv[3]
image_path = sys.argv[4]

class_names = [name.strip() for name in open(label_path).readlines()]

if net_type == 'mb2-ssd-lite':
    net = create_mobilenetv2_ssd_lite(len(class_names), is_test=True)
else:
    print("The net type is wrong. Only mb2-ssd-lite supported")
    sys.exit(1)
net.load(model_path)

if net_type == 'mb2-ssd-lite':
    predictor = create_mobilenetv2_ssd_lite_predictor(net, candidate_size=200)
else:
    print("The net type is wrong. Only mb2-ssd-lite supported")
    sys.exit(1)

orig_image = cv2.imread(image_path)

image = cv2.cvtColor(orig_image, cv2.COLOR_BGR2RGB)
boxes, labels, probs = predictor.predict(image, 10, 0.4)


cv2.line(orig_image, CENTER_LINEx1y1, CENTER_LINEx2y2, (0, 0, 0))

for i in range(boxes.size(0)):
    box = boxes[i, :]
    cv2.rectangle(orig_image, (box[0], box[1]), (box[2], box[3]), (255, 255, 0), 4)

    #If an obstacle is detected, find the angle from the center
    # if class_names[labels[i]] == 'obstacle':
        # interiorAngle = int(obstacleDetected(orig_image, box[0], box[1], box[2], box[3]))
    angle = detectObjects(orig_image, box[0], box[1], box[2], box[3])
    print(f"{class_names[labels[i]]}: {probs[i]:.2f}, {angle}{chr(176)}")
    label = f"{class_names[labels[i]]}: {probs[i]:.2f}, {angle}"

    cv2.putText(orig_image, label,
                (box[0] + 20, box[1] + 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,  # font scale
                (255, 0, 255),
                1)  # line type

path = "run_ssd_example_output.jpg"
cv2.imwrite(path, orig_image)
print(f"Found {len(probs)} objects. The output image is {path}")