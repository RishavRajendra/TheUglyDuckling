from gridMovement import GridMovement
from grid import Grid
from commandThread import CommandThread
from listenerThread import ListenerThread
import queue, time, serial, threading

ser = serial.Serial('/dev/ttyACM0', 9600, timeout=2)
time.sleep(1)

label1 = (0,1)
label2 = (1,1)
label3 = (1,2)
label4 = (1,4)
label5 = (5,3)
label6 = (6,6)

in_q = queue.Queue()
lock = threading.Lock()
cThread = CommandThread(in_q, ser, lock)
lThread = ListenerThread(in_q, ser, lock)
cThread.start()
lThread.start()
grid = Grid(8,8)
# grid.obstacles = [label1, label2,label3, label4]
# grid.obstacles = [label1, label2, label3, label4, label5, label6]
movement = GridMovement(grid, in_q)

movement.find_path()
movement.follow_path()
# in_q.put(['gridMove', (b'\x22', b'\x55')], True, 0.05)
# time.sleep(.5)
# in_q.put(['gridMove', (b'\x0A', b'\x55')], True, 0.05)


while not in_q.empty():
	pass

cThread.join()
lThread.join()