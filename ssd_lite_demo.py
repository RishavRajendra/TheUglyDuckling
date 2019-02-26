#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = "Rishav Rajendra"
__license__ = "MIT"
__status__ = "Development"

from vision.mobilenetv2_ssd_lite import create_mobilenetv2_ssd_lite, create_mobilenetv2_ssd_lite_predictor
from vision.utils.misc import Timer
import cv2
import sys

import warnings
warnings.filterwarnings('ignore')

if len(sys.argv) < 4:
    print('Usage: python ssd_lite_demo.py <net type> <model path> <label path> [video file]')
    sys.exit(0)
net_type = sys.argv[1]
model_path = sys.argv[2]
label_path = sys.argv[3]

if len(sys.argv) >= 5:
    cap = cv2.VideoCapture(sys.argv[4]) #From file
else:
    cap = cv2.VideoCapture(0) #From camera
    cap.set(3, 1920)
    cap.set(4, 1080)

class_names = [name.strip() for name in open(label_path).readlines()]
num_classes = len(class_names)

if net_type == 'mb2-ssd-lite':
    net = create_mobilenetv2_ssd_lite(len(class_names), is_test=True)
else:
    print("Only mb2-ssd-lite supported right now")
    sys.exit(1)
net.load(model_path)

if net_type == 'mb2-ssd-lite':
    predictor = create_mobilenetv2_ssd_lite_predictor(net, candidate_size=200)
else:
    print("Only mb2-ssd-lite supported right now")
    sys.exit(1)

timer = Timer()
while True:
        ret, orig_image = cap.read()
        if orig_image is None:
                continue
        image = cv2.cvtColor(orig_image, cv2.COLOR_BGR2RGB)
        timer.start()
        boxes, labels, probs = predictor.predict(image, 10, 0.4)
        interval = timer.end()
        print('Time: {:.2f}s, Detect Objects: {:d}.'.format(interval, labels.size(0)))
        for i in range(boxes.size(0)):
                box = boxes[i, :]
                label = f"{class_names[labels[i]]}: {probs[i]:.2f}"
                cv2.rectangle(orig_image, (box[0], box[1]), (box[2], box[3]), (255, 255, 0), 4)

                cv2.putText(orig_image, label,
                                        (box[0]+20, box[1]+40),
                                        cv2.FONT_HERSHEY_SIMPLEX,
                                        1,  # font scale
                                        (255, 0, 255),
                                        2)  # line type
        cv2.imshow('annotated', orig_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
                break
cap.release()
cv2.destroyAllWindows()
