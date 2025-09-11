import pyax12.connection
from enum import Enum
import serial
from time import time

LIDAR_READ_INTERVAL = 100


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

    def __init__(self, motor_port, motor_baud_rate, motor_left, motor_right, lidar_port, lidar_baud_rate=115200):
        self.__motor_left = motor_left
        self.__motor_right = motor_right
        self.__motor_port = motor_port
        self.__motor_baud_rate = motor_baud_rate
        self.__lidar_port = lidar_port
        self.__lidar_baud_rate = lidar_baud_rate
        self.__last_lidar_read = 0
        self.__lidar_response = [0x4D, 0x46, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xFF, ]

    def __enter__(self):
        # Enter the runtime context for the with statement.
        # setup sensors
        print(f"Entering context for {self}")
        self.__motors = pyax12.connection.Connection(port=self.__motor_port, baudrate=self.__motor_baud_rate)
        self.__lidar = serial.Serial(self.__lidar_port, self.__lidar_baud_rate, timeout=5, writeTimeout=5)
        self.__lidar.flush()
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
        if time() - self.__last_lidar_read > LIDAR_READ_INTERVAL:
            self.__lidar.flush()
            response = self.__lidar.read(20)

            if len(response) != 20 or response[:2] != [0x4D, 0x46]:
                self.__lidar.flush()
                return

            self.__lidar_response = response
            self.__last_lidar_read = time()

    def get_lidar(self, direction):
        try:
            if direction in LidarDir:
                return (self.__lidar_response[direction.value+2] << 8) + self.__lidar_response[direction.value+4]
        except TypeError:
            if 0 <= direction < 8:
                return (self.__lidar_response[direction + 2] << 8) + self.__lidar_response[direction + 4]

    def get_heading(self):
        # remember to include url of compass code base here
        pass
