# Functions the Raspberry Pi can call to communicate movement to the arduino
import queue, threading
import serial, time

# Connect to arduino
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=.1)
time.sleep(1)

# direction bytes
fwd = b'\x88'
rev = b'\x22'
rotl = b'\xAA'
rotr = b'\x00'
strl = b'\xA0'
strr = b'\x0A'

# motion bytes
allmotors = b'\x55'
right = b'\x05'
left = b'\x50'
front = b'\x11'
rear = b'\x44'
right45 = b'\x14'
left45 = b'\x41'

def motionThread(threading.Thread):

    def __init__(self, queue):
        super(None, self).__init__()
        self.func = { 'move': self.move,
                      'turn': self.turn,
                      'move45': self.move45}
        self.queue = queue
        self.stroprequest = threading.Event()

    def run(self):
        while not self.stoprequest.isSet():
            work = self.queue.get(True, 0.05)
            self.func[work[0]](work[1])

    # Moves robot X distance (in centimenters) in direction given
    def move(args):
        byteArr = b'\x00' + args[0]+bytes([args[1]])+b'\x00'
        # print(byteArr)
        ser.write(byteArr)

    # Turn robot by specified degrees
    # only use rotl or rotr for direction.
    def turn(args):
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

    def move45(args):
        byteArr = b'\x03'+args[0]+bytes([args[1]])+args[2]
        ser.write(byteArr)