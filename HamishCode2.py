# look at the distance to the furthest block infront, divide that number of 28 to get the number of blocks
# drive forwards until that number reaches < 28
# check left and right for a free space, turns to the open square 
# if none drive backwards
# while this is happening make sure that the front right and front left sensors are the same output
import time
from pyax12.connection import Connection
import math
from icm20948 import ICM20948
import RPi.GPIO as GPIO
import serial
from random import randint
from enum import Enum
#this is for the lidar sensor
lidar_port = "/dev/ttyACM1"
lidar_baud_rate = 115200
lidar = serial.Serial(lidar_port, lidar_baud_rate, timeout=1, writeTimeout=1)
lidar.flushInput()
lidar.flushOutput()

lidar_offset = [3.1,1.7,1.8,2.4,1,2.4,1.8,2.3]
LIDAR_READ_INTERVAL = 0.5
last_read = 0
lidar_response = bytearray(20)


class SearchStates(Enum):
    SEARCH_NODE = 0
    DISCOVER_NODES = 1
    PICK_NEXT_NODE = 2,
    MOVE_NODE = 3

# this is for the visited squares
direction_facing = 0
unexplored_nodes = []
connections = {}
visited = set((0, 0))
current_node = [0, 0]
search_state = SearchStates.DISCOVER_NODES
target_square = [0, 0]


# Square Size 

SQUARE_SIZE = 28

# this is for the button
BUTTON_PIN = 26
GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
motors_running = False
last_button_state = GPIO.input(BUTTON_PIN)

# Servo connection
serial_connection = Connection(port="/dev/ttyACM0", baudrate=1000000)

# Servo IDs
left_motor_id = 15
right_motor_id = 18

# how long it is turning for, remove this once the compass works 
turn_time = 2.0

# servos speed
motor_speed = 480
MIN_SPEED = 100
MAX_DISTANCE = 20
MIN_DISTANCE = 0

MAX_ANGLE = 45
MIN_ANGLE = 5
MAX_TURN_SPEED = 480
MIN_TURN_SPEED = 200

# Enable torque
serial_connection.write_data(left_motor_id, 24, [1])
serial_connection.write_data(right_motor_id, 24, [1])

# IMU setup
imu = ICM20948()

# Axis indices
X = 0
Y = 1
Z = 2

AXES = X, Y

amin = [-70.95, -31.80, -89.70]
amax = [23.25, 68.25, 11.25]


# Function to control a wheel
def wheel(motor_id, speed, direction="cw"):
    speed = max(min(speed, 1023), 0) 
    if direction == "cw":
        value = speed  # 0–1023
    else:
        value = 1024 + speed  # 1024–2047 means CCW
    serial_connection.write_data(motor_id, 32, [value & 0xFF, (value >> 8) & 0xFF])

def drive_speed(distance):
    if distance > MAX_DISTANCE:
        return motor_speed

    # get slower as you get closer to goal
    return max(distance-MIN_DISTANCE, 0)/(MAX_DISTANCE-MIN_DISTANCE) * (motor_speed-MIN_SPEED) + MIN_SPEED


# Stop motor
def stop_motor(motor_id):
    serial_connection.write_data(motor_id, 32, [0, 0])

def forwards(speed = None):
	wheel(left_motor_id, motor_speed if speed == None else int(speed), "cw")
	wheel(right_motor_id, motor_speed if speed == None else int(speed), "ccw")

def backwards(speed = None):
	wheel(left_motor_id, motor_speed if speed == None else int(speed), "ccw")
	wheel(right_motor_id, motor_speed if speed == None else int(speed), "cw")

# Turn towards a target angle
def turn(target_angle, heading):
    diff = (target_angle - heading + 360) % 360  
    speed = min(max(abs(diff-MIN_ANGLE), 0), (MAX_ANGLE-MIN_ANGLE))/(MAX_ANGLE-MIN_ANGLE) * (MAX_TURN_SPEED-MIN_SPEED) + MIN_SPEED
    speed = int(speed)
    if diff > 180:
        # Turn CCW
        wheel(left_motor_id, speed, "ccw")
        wheel(right_motor_id, speed, "ccw")
    else:
        # Turn CW
        wheel(left_motor_id, speed, "cw")
        wheel(right_motor_id, speed, "cw")

target_angle = 180
aligned = False

def read_lidar():
	lidar.flushInput()
	lidar.flushOutput()
	response = lidar.read(20)

	lidar_response = response
	distances = []

	for i in range(8):
		idx = i*2 + 2
		dist = ((lidar_response[idx] << 8) + lidar_response[idx+1])/10 - lidar_offset[i]
		distances.append(dist)
	for i in range(len(distances)):
		distances[i] = round(distances[i], 2)
	return distances

def get_heading():
	# Read the current, uncalibrated, X, Y & Z magnetic values from the magnetometer and save as a list
		mag = list(imu.read_magnetometer_data())

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
		heading = math.atan2(
				mag[AXES[0]],
				mag[AXES[1]])

		# If heading is negative, convert to positive, 2 x pi is a full circle in Radians
		if heading < 0:
			heading += 2 * math.pi

		# Convert heading from Radians to Degrees
		heading = math.degrees(heading)
		# Round heading to nearest full degree
		heading = round(heading)

		# Note: Headings will not be correct until a full 360 deg calibration turn has been completed to generate amin and amax data
		return heading

def squares_around():
	return [round((distance[0]/SQUARE_SIZE)-0.5, 0), round((distance[2]/SQUARE_SIZE)-0.5, 0), round((distance[4]/SQUARE_SIZE)-0.5, 0), round((distance[6]/SQUARE_SIZE)-0.5, 0)]


def move_to_square(square_number: int):
    # get values
    front = distance[0]
    back = distance[4]
    # calculate distances
    number_of_squares = squares_around()
    target_b = SQUARE_SIZE * (square_number + 0.5)
    target_f = (number_of_squares[0] + number_of_squares[2]+1) * SQUARE_SIZE - target_b
    # calculate how far front + back are, swapping first item to keep sign
    # consistent and then averaging
    diff_front = front - target_f
    diff_back = target_b - back
    avg_diff = (diff_front + diff_back) / 2
    print(avg_diff)
    print(target_b, target_f)
    

    if (abs(diff_front) < 3 and abs(diff_back) < 3) or (avg_diff < 5 and abs(diff_front) < 10 and abs(diff_back) < 10):
        stop_motor(15)
        stop_motor(18)
        return True
        
    elif math.copysign(1, diff_front) == math.copysign(1, diff_back):
        if diff_front > 0:
            forwards(drive_speed(abs(avg_diff)))
        else:
            backwards(drive_speed(abs(avg_diff)))
    
    else:
        dist = diff_back if abs(diff_back) < abs(diff_front) else diff_front
        if diff_front > 0:
            forwards(drive_speed(abs(dist)))
        else:
            backwards(drive_speed(abs(dist)))    
    return False

def discover_nodes():
	connected = []
	squares = squares_around()
	for i in range(0, 4):
		if squares[i] > 0:
			dirNode = (direction_facing + i) % 4
			if dirNode % 2 == 0: 
				connected.append([current_node[0], current_node[1] - dirNode + 1])
			else:
				connected.append([current_node[0] - dirNode + 2, current_node[1]])
			if tuple(connected[len(connected)-1]) not in visited:
				unexplored_nodes.append(connected[len(connected)-1])
				
	connections[tuple(current_node)] = connected
	
def get_next_node():
	# if it has explored all nodes go home
	if len(unexplored_nodes) == 0:
        return [0, 0]
    # get to the next unexplored node
    next_node = unexplored_nodes.pop()
    # make sure the node hasn't been visited yet
    while next_node in visited:
        if len(unexplored_nodes) == 0:
            return [0, 0]
        next_node = unexplored_nodes.pop()

    return next_node
try:
    while True:
        button_state = GPIO.input(BUTTON_PIN)

        if button_state == GPIO.LOW and last_button_state == GPIO.HIGH:
            motors_running = not motors_running
            target_angle = heading
            time.sleep(0.1)
        """mag = list(imu.read_magnetometer_data())
        heading = math.atan2(mag[AXES[0]], mag[AXES[1]])
        if heading < 0:
            heading += 2 * math.pi
        heading = math.degrees(heading)
        heading = round(heading)
        #print(f"Heading: {heading}° \t targets: {target_angle}")"""
        
        heading = get_heading()
        
        if time.time() - last_read > LIDAR_READ_INTERVAL:
            distance = read_lidar()
            print(distance)
            #print("Number of tiles ahead:/t", distance[0]/30, round((distance[0]/30)-0.5, 0)) 
            print(squares_around())
            last_read = time.time()
            
        # where we are, where we were, where we can go
            
            
        if motors_running:
            if search_state == SearchStates.SEARCH_NODE:
                pass
                
            elif search_state == SearchStates.DISCOVER_NODES:
                discover_nodes()
                search_state = SearchStates.PICK_NEXT_NODE
                
            elif search_state == SearchStates.PICK_NEXT_NODE:
                print(unexplored_nodes)
                get_next_node()
                search_state = SearchStates.MOVE_NODE
                
            elif search_state == SearchStates.MOVE_NODE:
                pass
                
            else:
                break
                    
        else:
            stop_motor(15)
            stop_motor(18)

finally:
    stop_motor(left_motor_id)
    stop_motor(right_motor_id)
    # Disable torque
    serial_connection.write_data(left_motor_id, 24, [0])
    serial_connection.write_data(right_motor_id, 24, [0])
    serial_connection.close()

