import serial
import keyboard
import time
import picamera
from subprocess import call

# connect to arduino
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=.1)
# might not need this
time.sleep(1)

# motion bytes
fwd = b'\x88'
rev = b'\x22'
rotl = b'\xAA'
rotr = b'\x00'
strl = b'\xA0'
strr = b'\x0A'

camera = picamera.PiCamera()

t_end = time.time() + 60 * 0.5
camera.start_recording("testVideo.h264")
while time.time() < t_end:
	if keyboard.is_pressed('w'):
		ser.write(fwd)
	elif keyboard.is_pressed('s'):
		ser.write(rev)
	elif keyboard.is_pressed('a'):
		ser.write(strl)
	elif keyboard.is_pressed('d'):
		ser.write(strr)
	elif keyboard.is_pressed('q'):
		ser.write(rotl)
	elif keyboard.is_pressed('e'):
		ser.write(rotr)
	time.sleep(.2)
camera.start_recording()

print("Converting video")
command = "MP4Box -add testVideo.h264 convertedVideo.mp4"
call([command], shell=True)
print("Video converted")
