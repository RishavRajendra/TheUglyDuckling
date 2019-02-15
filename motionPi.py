# Functions the Raspberry Pi can call to communicate movement to the arduino
import queue, threading
import serial, time

# Connect to arduino
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=.1)
time.sleep(1)

# direction bytes
fwd = b'\xA0'
rev = b'\x0A'
rotl = b'\x00'
rotr = b'\xAA'
strl = b'\x22'
strr = b'\x88'

# motion bytes
allmotors = b'\x55'
right = b'\x05'
left = b'\x50'
front = b'\x11'
rear = b'\x44'
right45 = b'\x14'
left45 = b'\x41'

class motionThread(threading.Thread):

    def __init__(self, queue):
        super(motionThread, self).__init__()
        self.func = { 'move': self.move,
                      'turn': self.turn,
                      'move45': self.move45}
        self.queue = queue
        self.stoprequest = threading.Event()

    def run(self):
        while not self.stoprequest.isSet():
            if not self.queue.empty():
                work = self.queue.get(True, 0.05)
                self.func[work[0]](work[1])
    def join(self, timeout=None):
        self.stoprequest.set()
        super(motionThread, self).join(timeout)

    # Movement instructions sent to Queue should follow
    # this format: ['function_name', (args)]
    # args format is function dependent.

    # Moves robot X distance (in centimenters) in direction given
    # (args) format: (direction, distance)
    def move(self,args):
        byteArr = b'\x00' + args[0]+bytes([args[1]])+b'\x00'
        ser.write(byteArr)

    # Turn robot by specified degrees
    # only use rotl or rotr for direction.
    # (args) format: (direction, degrees)
    def turn(self, args):
        byteArr = b'\x01' + args[0]+bytes([args[1]])+b'\x00'
        ser.write(byteArr) 

    # Move 45 degrees diagonally.
    ############################
    #   dir and motors combo
    ############################
    # To move:
    # NE = fwd, right45
    # NW = fwd, left45
    # SE = rev, left45
    # SW = rev, right45

    # (args) format: (direction, distance, motors)
    def move45(self, args):
        byteArr = b'\x02'+args[0]+bytes([args[1]])
        ser.write(byteArr)

    # Moves robot a predefined distance dependent on 
    # distance from center of tile1 to tile2

    # (args) format: (direction, motors)
    def gridMove(self, args):
        byteArr = b'\x03' + args[0] + bytes([args[1]])
        ser.write(byteArr)