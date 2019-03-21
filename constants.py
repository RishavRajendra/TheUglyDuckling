# direction bytes
fwd = b'\xA0'
rev = b'\x0A'
rotl = b'\x00'
rotr = b'\xAA'
strl = b'\x22'
strr = b'\x88'

# Our neural network is trained on 300 x 300 images
CAMERA_RESOLUTION = (300, 300)
CAMERA_FRAMERATE = 10

#Assuming that the picture is 300x300
CENTER_LINEx1y1 = (150, 0)
CENTER_LINEx2y2 = (150, 300)

# The calculated sweet spot the grabber can pickup successfully
PICKUP_SWEET_SPOTx1y1 = (164, 0)
PICKUP_SWEET_SPOTx2y2 = (164, 259)

# Focal length of a rasperry pi v2 camera
FOCAL_LENGTH = 3.04

# M = avg(m_x, m_y) / FOCAL_LENGTH
PIXEL_PER_MM = 102.3

# Magic number to compensate for labelling.
ERROR_VAL = 3.3

# Height of object
OBSTACLE_HEIGHT = 35
TARGET_HEIGHT = 15

# Distance from beginning of camera range to center of robot
CENTER_DISTANCE_UP = 6.5
CENTER_DISTANCE_DOWN = 6

