# Functions the Raspberry Pi can call to communicate movement to the arduino
import queue, threading
import serial, time

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

class CommandThread(threading.Thread):


    # (queue) = Queue()
    # (serial) = serial connection to arduino
    # (lock) = threading.Lock()
    def __init__(self, queue, serial, lock):
        super(CommandThread, self).__init__()
        self.func = { 'move': self.move,
                      'turn': self.turn,
                      'move45': self.move45,
                      'gridMove': self.gridMove}
        self.queue = queue
        self.serial = serial
        self.lock = lock
        self.stoprequest = threading.Event()

    def run(self):
        while not self.stoprequest.isSet():
            if not self.queue.empty():
                self.lock.acquire(True)
                # time.sleep(1)
                print("motion send")
                work = self.queue.get(True, 0.05)
                self.func[work[0]](work[1])
                self.lock.release()
            time.sleep(.2)
    def join(self, timeout=None):
        self.stoprequest.set()
        super(CommandThread, self).join(timeout)

    # Movement instructions sent to Queue should follow
    # this format: ['function_name', (args)]
    # args format is function dependent.

    # Moves robot X distance (in centimenters) in direction given
    # (args) format: (direction, distance)
    def move(self,args):
        byteArr = b'\x00' + args[0]+bytes([args[1]])+b'\x00'
        self.serial.write(byteArr)

    # Turn robot by specified degrees
    # only use rotl or rotr for direction.
    # (args) format: (direction, degrees)
    def turn(self, args):
        byteArr = b'\x01' + args[0]+bytes([args[1]])+b'\x00'
        self.serial.write(byteArr) 

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
        byteArr = b'\x02'+args[0]+bytes([args[1]]) + args[2]
        self.serial.write(byteArr)

    # Moves robot a predefined distance dependent on 
    # distance from center of tile1 to tile2

    # (args) format: (direction, motors)
    def gridMove(self, args):
        byteArr = b'\x03' + args[0] + args[1] + b'\x00'
        self.serial.write(byteArr)

    def armsMove(self, args):
        byteArr = b'\x04' 