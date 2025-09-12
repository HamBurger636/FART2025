"""import pyax12.connection
from enum import Enum
import serial
from time import time

LIDAR_READ_INTERVAL = 100


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
        if (time() - self.__last_lidar_read) > LIDAR_READ_INTERVAL:
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
"""

"""
#!/usr/bin/env python

import serial
import time

s = serial.Serial()                        # create a serial port object
s.baudrate = 1000000                        # baud rate, in bits/second
s.port = "/dev/ttyACM0"            # this is whatever port your are using
s.timeout = 3.0
s.open()


def jointMode(ID):
        s.write('W'+'j'+chr(ID))


def wheelMode(ID):
        s.write('W'+'w'+chr(ID))

def readDxl(ID,cmd):
        #j m or s
        output = ""
        s.write('R'+str(cmd)+chr(ID))
        for line in s.readline():
                output = output + line
        return int(output[-3:])        

def moveJoint(ID, position, speed):
        #move to position between 0-1024
        #512 is 12 o'clock
        #WRITE ID
        s.write('W'+'p'+chr(ID))

        #write position        
        position = int(position)
        s.write(chr(int(position)%256))
        s.write(chr(int(position)>>8))
        
        #WRITE SPEED
        velocity = int(speed)
        s.write(chr(int(velocity)%256))
        s.write(chr(int(velocity)>>8))

        #READ POSITION
        time.sleep(0.5)
        print(readDxl(ID,"j"))
        
def moveWheel(ID, speed):
    # Send command prefix (example: 'W', 's', ID)
    s.write(b'W')
    s.write(b's')
    s.write(bytes([ID]))
    
    # WRITE ADDRESS (Goal Position for continuous rotation)
    addr = 32
    s.write(bytes([addr & 0xFF]))      # low byte
    s.write(bytes([addr >> 8]))        # high byte
    
    # WRITE SPEED / direction
    if speed >= 0:
        speed_value = int(speed)       # CW: 0–511
    else:
        speed_value = 512 + int(-speed)  # CCW: 513–1023
    
    s.write(bytes([speed_value & 0xFF]))  # low byte
    s.write(bytes([speed_value >> 8]))    # high byte

while True:
    moveWheel(15,255)"""
    
    
"""    
import RPi.GPIO as GPIO
import time
from Ax12 import Ax12

# =====================
# GPIO setup
# =====================
BUTTON_PIN = 26
GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# =====================
# Dynamixel setup
# =====================
Ax12.DEVICENAME = '/dev/ttyACM0'  # adjust if needed
Ax12.BAUDRATE = 1_000_000
Ax12.connect()

LEFT_ID = 15
RIGHT_ID = 18
dxl_left = Ax12(LEFT_ID)
dxl_right = Ax12(RIGHT_ID)

motor_speed = 600

# Default motor speeds (right motor reversed)
dxl_left.set_moving_speed(0)
dxl_right.set_moving_speed(0)

wait = 3


motors_running = False
last_button_state = GPIO.input(BUTTON_PIN)

# =====================
# Main interactive function
# =====================


def spin(speed):
    dxl_left.set_moving_speed(speed)
    dxl_right.set_moving_speed(speed)

    
def stop(speed):
    dxl_left.set_moving_speed(0)
    dxl_right.set_moving_speed(0)



# =====================
# Button toggle loop
# =====================
try:
    while True:
        button_state = GPIO.input(BUTTON_PIN)

        if button_state == GPIO.LOW and last_button_state == GPIO.HIGH:
            motors_running = not motors_running
            print("motor", motors_running)
        
        if motors_running:
            time.sleep(0.4)
            spin(motor_speed)
            time.sleep(0.4)
            stop(motor_speed)
        else:
            stop(motor_speed)


        last_button_state = button_state


except KeyboardInterrupt:
    print("\nExiting program")

finally:
    # Safely stop motors and disconnect
    dxl_left.set_moving_speed(0)
    dxl_right.set_moving_speed(0)
    dxl_left.set_torque_enable(0)
    dxl_right.set_torque_enable(0)
    Ax12.disconnect()
    GPIO.cleanup()
    print("Motors stopped and GPIO cleaned up.")
"""
"""
import time
from pyax12.connection import Connection

# Connect to the serial bus
serial_connection = Connection(port="/dev/ttyACM0", baudrate=1000000)

# Servo IDs
left_motor_id = 15
right_motor_id = 18

# Enable torque (address 24)
serial_connection.write_data(left_motor_id, 24)
serial_connection.write_data(right_motor_id, 24)

def wheel_mode_spin(motor_id, speed):
    
    if direction == "cw":
        value = speed  # 0–1023
    else:
        value = 1024 + speed  # 1024–2047 means CCW
    serial_connection.write_data(motor_id, 32, [value & 0xFF, (value >> 8) & 0xFF])

def stop_motor(motor_id):
    serial_connection.write_data(motor_id, 32, [0, 0])

try:
    # Spin both motors forward (CW) for 5s
    wheel_mode_spin(left_motor_id, 300)
    wheel_mode_spin(right_motor_id, 300)
    time.sleep(5)

    # Spin both motors backward (CCW) for 5s
    wheel_mode_spin(left_motor_id, 300)
    wheel_mode_spin(right_motor_id, 300)
    time.sleep(5)

    # Stop both motors
    stop_motor(left_motor_id)
    stop_motor(right_motor_id)

    # Disable torque
    serial_connection.write_data(left_motor_id, 24, [0])
    serial_connection.write_data(right_motor_id, 24, [0])

    # Close connection
    serial_connection.close()

finally:
    # Spin both motors backward (CCW) for 5s
    wheel_mode_spin(left_motor_id, 0)
    wheel_mode_spin(right_motor_id, 0)
"""

import serial
import time

# Serial port for your LiDAR (change as needed)
lidar_port = "/dev/ttyACM1"
baud_rate = 115200

# Open serial connection
lidar = serial.Serial(lidar_port, baud_rate, timeout=1, writeTimeout=1)
lidar.flushInput()
lidar.flushOutput()

lidar_offset = [0]*8  # optional offsets for each direction
LIDAR_READ_INTERVAL = 0.5
last_read = 0
lidar_response = bytearray(20)

while True:
    if time.time() - last_read > LIDAR_READ_INTERVAL:
        lidar.flushInput()
        lidar.flushOutput()
        response = lidar.read(20)

        if len(response) != 20 or response[:2] != bytearray([0x4D, 0x46]):
            print("Bad read")
            continue

        lidar_response = response
        distances = []

        for i in range(8):
            idx = i*2 + 2
            dist = ((lidar_response[idx] << 8) + lidar_response[idx+1])/10 - lidar_offset[i]
            distances.append(dist)

        print(distances)
        last_read = time.time()
