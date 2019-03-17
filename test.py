import nav.gridMovement
import nav.grid
import queue, serial, time, math
from nav.command import Command

def corrected_angle(angle, dist):
    angle = 180 - angle
    a = math.sqrt(math.pow(dist,2) + math.pow(3.5, 2) - 2*dist*3.5*math.cos(math.radians(angle)))
    angle_c = math.asin(math.sin(math.radians(angle))*dist/a)
    return math.ceil(180-angle-math.degrees(angle_c))
angle = 0
dist = 0

ser = serial.Serial('/dev/ttyACM0', 9600, timeout=2)
time.sleep(1)
q = queue.Queue()
grid = nav.grid.Grid(8,8)
movement = nav.gridMovement.GridMovement(grid, q)
commands = Command(q, ser)



#Raise arms and lower arms
q.put(['drop',(0)])
commands.execute()
time.sleep(2)
q.put(['pickup',(0)])
commands.execute()



