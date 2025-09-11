import pyax12.connection
from enum import Enum
import serial
from time import time
import math
from icm20948 import ICM20948

LIDAR_READ_INTERVAL = 0.5
#offsets
amin=[-87.0, -43.8, -79.5]
amax=[8.1, 44.1, 12.0]

X = 0
Y = 1
Z = 2

# The two axes which relate to heading, depends on orientation of the sensor
# Think Left & Right, Forwards and Back, ignoring Up and Down
AXES = Y, X


class Directions(Enum):
    NORTH = 0,
    EAST = 1,
    SOUTH = 2,
    WEST = 3


class LidarDir(Enum):
    FRONT = 0
    FRONT_RIGHT = 1
    RIGHT = 2
    BACK_RIGHT = 3
    BACK = 4
    BACK_LEFT = 5
    LEFT = 6
    FRONT_LEFT = 7


class FART:

    def __init__(self, motor_port, motor_baud_rate, motor_left, motor_right, lidar_port, lidar_baud_rate=115200, lidar_offset=[0,0,0,0,0,0,0,0]):
        self.__motor_left = motor_left
        self.__motor_right = motor_right
        self.__motor_port = motor_port
        self.__motor_baud_rate = motor_baud_rate
        self.__lidar_port = lidar_port
        self.__lidar_baud_rate = lidar_baud_rate
        self.__last_lidar_read = 0
        self.__lidar_response = [0x4D, 0x46, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xFF, ]
        self.lidar_offset=lidar_offset

    def __enter__(self):
        # Enter the runtime context for the with statement.
        # setup sensors
        print(f"Entering context for {self}")
        self.__motors = pyax12.connection.Connection(port=self.__motor_port, baudrate=self.__motor_baud_rate)
        self.__lidar = serial.Serial(self.__lidar_port, self.__lidar_baud_rate, timeout=1, writeTimeout=1)
        self.__lidar.flushInput()
        self.__lidar.flushOutput()
        self.__imu = ICM20948()

        # setup compass
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        # Exit the runtime context for the with statement.
        # clean up Serial and other connections
        self.stop()
        self.__motors.close()
        self.__lidar.close()
        return False

    def __move_raw__(self, left, right):
        # speed should be between -100 and 100
        bounded_left = min(100, max(-100, left))
        bounded_right = min(100, max(-100, right))
        self.__motors.set_speed(self.__motor_left, bounded_left)
        self.__motors.set_speed(self.__motor_right, bounded_right)

    def stop(self):
        self.__move_raw__(0, 0)

    def forward(self, speed):
        pass

    def turn(self, speed):
        pass

    def read_lidar(self):
        if (time() - self.__last_lidar_read) > LIDAR_READ_INTERVAL:
            self.__lidar.flushInput()
            self.__lidar.flushOutput()
            response = self.__lidar.read(20)

            if len(response) != 20 or response[:2] != bytearray([0x4D, 0x46]):
                print("bad read")
                self.__lidar.flushInput()
                self.__lidar.flushOutput()
                return
            
            self.__lidar_response = response
            print(response)
            self.__last_lidar_read = time()

    def get_lidar(self, direction):
        try:
            if direction in LidarDir:
                print(direction.value*2+2, direction.value*2+3)
                return ((self.__lidar_response[direction.value*2+2] << 8) + self.__lidar_response[direction.value*2+3])/10 - self.lidar_offset[direction.value]
        except TypeError:
            if 0 <= direction < 8:
                return ((self.__lidar_response[direction*2+2] << 8) + self.__lidar_response[direction*2+4])/10 - self.lidar_offset[direction]

    def get_heading(self):
        # remember to include url of compass code base here
        mag = list(self.__imu.read_magnetometer_data())

        # Step through each uncalibrated X, Y & Z magnetic value and calibrate them the best we can
        for i in range(3):
            v = mag[i]
            # If our current reading (mag) is less than our stored minimum reading (amin), then save a new minimum reading
            # ie save a new lowest possible value for our calibration of this axis
            if v < amin[i]:
                amin[i] = v
            # If our current reading (mag) is greater than our stored maximum reading (amax), then save a new maximum reading
            # ie save a new highest possible value for our calibration of this axis
            if v > amax[i]:
                amax[i] = v

            # Calibrate value by removing any offset when compared to the lowest reading seen for this axes
            mag[i] -= amin[i]

            # Scale value based on the highest range of values seen for this axes
            # Creates a calibrated value between 0 and 1 representing magnetic value
            try:
                mag[i] /= amax[i] - amin[i]
            except ZeroDivisionError:
                pass
            # Shift magnetic values to between -0.5 and 0.5 to enable the trig to work
            mag[i] -= 0.5

        # Convert from Gauss values in the appropriate 2 axis to a heading in Radians using trig
        # Note this does not compensate for tilt
        heading = math.atan2(mag[AXES[0]], mag[AXES[1]])

        # If heading is negative, convert to positive, 2 x pi is a full circle in Radians
        if heading < 0:
            heading += 2 * math.pi

        # Convert heading from Radians to Degrees
        return math.degrees(heading)
        
