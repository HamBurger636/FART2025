#!/usr/bin/env python3
# hamishCode2.py

# these are all of the libs needed for the software
import time
import math
from enum import Enum
from random import randint
import subprocess
import atexit
import os

# these are the hardware libs
from pyax12.connection import Connection
from icm20948 import ICM20948
import RPi.GPIO as GPIO
import serial


# Variables 

# This is for the lidar where it defins the communication protcals and the setup
LIDAR_PORT = "/dev/ttyACM1"
LIDAR_BAUD = 115200
LIDAR_READ_INTERVAL = 0.25

LIDAR_OFFSET = [3.1, 1.7, 1.8, 2.4, 1.0, 2.4, 1.8, 2.3]  # per-sensor offsets because they are all conststantally off by a set amount 
SQUARE_SIZE = 29.5  # tile size, this is because the tiles migth be slightly diffrent sizes on the day
FRONT_IDX = 0
RIGHT_IDX = 2
BACK_IDX = 4
LEFT_IDX = 6


# There is a button which starts and stops the moving and the mapping
BUTTON_PIN = 26
GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# the servos are dynamixal Ax12 with the id's 15 and 18. Their speed works by 0-1024 being clock wise and 1025 - 2058 being counter clock wise
# the speeds are determinded by the distance of the robot to a wall to stop it over shooting, the max speed is the max speed which it is able to move, i chose this speed because it is not too fast and unalbe to over shoot, the min speed is needed becasaue at around 200 the motors are unable to move the robot 
SERVO_PORT = "/dev/ttyACM0"
SERVO_BAUD = 1000000
LEFT_MOTOR_ID = 15
RIGHT_MOTOR_ID = 18
DEFAULT_MOTOR_SPEED = 480
MIN_SPEED = 300


# This is the speeds needed for turning and tolerance is how close the robot has to be to the correct angle to be deemed correct
TURN_TOLERANCE_DEG = 5
TURN_MAX_SPEED = 480
TURN_MIN_SPEED = 300

# this code was not used in the final robot as of the 9/10/2025 because it was able to center itself well enough to not needed it but it worked by making the diagonals equal 
# straightening thresholds
STRAIGHT_DIFF_THRESHOLD = 3.0  # cm difference left vs right considered "not straight"
STRAIGHT_ANGLE_ADJUST = 6.0  # degrees to nudge by during straightening
STRAIGHT_PD_P = 6.0
STRAIGHT_PD_D = 2.0

# the color sensor gives back three values (r, g, b), and if it sees white it continues and black moves backwards. These values might need to be changed on the day
BLACK_THRESHOLD = 10
WHITE_THRESHOLD = 30

# the states the program can be in
motors_running = False
last_button_state = GPIO.input(BUTTON_PIN)

# this establishes a connectoin with the servos
serial_connection = Connection(port=SERVO_PORT, baudrate=SERVO_BAUD)

# when starting you need to enable torque on the motors becasue it is not on by default and is needed to get them to move
serial_connection.write_data(LEFT_MOTOR_ID, 24, [1])
serial_connection.write_data(RIGHT_MOTOR_ID, 24, [1])

# IMU for the lidar sensor
imu = ICM20948()

# LIDAR serial connections 
lidar = serial.Serial(LIDAR_PORT, LIDAR_BAUD, timeout=0.5)
lidar.flushInput()
lidar.flushOutput()
last_lidar_read = 0
# there is 8 sensors used around the robot 
distance = [0.0] * 8

# The color sensor is a DFRobot color sensor and it is attached to a Arduino Nano which communicats through the serial ports with the raspberry pi  
colourSensor = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.5)

# the communicatoin with the arduino was slowing down the program because once the main program called for the RGB values it took half a second for the arduino to reply in which time nothing was able happen, because of this i moved the communicatoin over to a diffrent python file which prints the vlaues into a text file which the main file reads  
# this is for opening the other program which checks for the colours 
COLOUR_HELPER_PATH = "/home/fart/fart/colorsensor.py"  # adjust path

# This should start the colour helper in the background but it does not work
colour_proc = subprocess.Popen(
    ["python3", COLOUR_HELPER_PATH],
    stdout=subprocess.DEVNULL,
    stderr=subprocess.DEVNULL
)

# This it to change the navagiation states
class SearchStates(Enum):
    SEARCH_NODE = 0
    DISCOVER_NODES = 1
    PICK_NEXT_NODE = 2
    TURN_TO_ANGLE = 3
    MOVE_NODE = 4
    STRAIGHTEN = 5

# these are more needed vaiables
direction_facing = 0  # 0: north (set when the button is pressed), 1: east, 2: south, 3: west
current_node = [0, 0]  # x,y
visited = set([tuple(current_node)])

# blocked is if the square is a black square
blocked = set()  
unexplored_nodes = []

# the connections are all of the squares that are able to be moved from one to another, connections are only made to adjacent squares
connections = {}  # adjacency: tuple(node) -> list of neighbor tuples
search_state = SearchStates.DISCOVER_NODES
target_node = [0, 0]
target_path = []
path_idx = 0

# target angles for facing directions (calibrate these at the start), i am doing this instead of just adding or removing 90 from the current facing becasue there will be a slow drift in direction over many turns
# they are normally not set here but instead you face the robot every directoin and press the button at the start 4 times to set every direction
target_angles = [348, 75, 155, 244]  # 0:north,1:east,2:south,3:west


# These functions are used to move to robot using the servos 

# every functions calls wheel which writes the serial with the information given from the functions and communicates through serial to the motors 
def wheel(motor_id, speed, direction="cw"):
    # Write to AX12 style motor: speed range 0-1023 for CW and 1024 - 2058 for CCW
    speed = max(0, min(1023, int(speed)))
    if direction == "cw":
        value = speed
    else:
        value = 1024 + speed
    serial_connection.write_data(motor_id, 32, [value & 0xFF, (value >> 8) & 0xFF])

# this bipasses the wheel functions and kills whatever motor you want
def stop_motor(motor_id):
    serial_connection.write_data(motor_id, 32, [0, 0])

# these are the functoins for moving the robot 
def forwards(speed=None):
    # the speed is a functoins which makes the robot slow down if it is close to a wall
    s = DEFAULT_MOTOR_SPEED if speed is None else int(speed)
    wheel(LEFT_MOTOR_ID, s, "cw")
    wheel(RIGHT_MOTOR_ID, s, "ccw")
# it is the same for backwards as forwards just reversed directins
def backwards(speed=None):
    s = DEFAULT_MOTOR_SPEED if speed is None else int(speed)
    wheel(LEFT_MOTOR_ID, s, "ccw")
    wheel(RIGHT_MOTOR_ID, s, "cw")

# if i want to use stop both of the motors i use this functoin
def stop_all():
    stop_motor(LEFT_MOTOR_ID)
    stop_motor(RIGHT_MOTOR_ID)
    
    
# LIDAR & IMU utilities
# the lidar is a terra bee Tera ranger 
def read_lidar():
    # Read 20 bytes frame as original code, parse safely and return list of 8 distances (cm)."""
    global lidar
    # it gives errors until it is set up which can cause problems so it needs a try 
    try:
	# this cleans the inputs getting it ready for a new string
        lidar.flushInput()
	# there is 20 bytes that the terra ranger gives
        resp = lidar.read(20)
        if len(resp) < 20:
            # partial read: return previous distances unchanged
            return distance
        # This changes the raw data into 8 cm values
        d = []
        for i in range(8):
            idx = 2 + i*2
            raw = (resp[idx] << 8) + resp[idx+1]
            cm = raw / 10.0 - LIDAR_OFFSET[i]
            d.append(round(max(0.0, cm), 2))
        return d
    # this handles any errors on the start up
    except Exception as e:
        print("LIDAR read error:", e)
        return distance

# The comapss heading with a min/max calibration
amin = [-71.40, -30.45, -89.25]
amax = [24.45, 70.05, 13.65]
# the last 10 digets are averaged out so if there is a random outlier it dosnt throw off the whole system
heading_history = []

# this is used to get the heading of the compass 
def get_heading_single():
    # mag is just a list of the 3 raw magnetometer values 
    mag = list(imu.read_magnetometer_data())
    # for each of the values it sets them inside the upper and lower bounds, this was taken off their website
    for i in range(3):
        v = mag[i]
        if v < amin[i]:
            amin[i] = v
        if v > amax[i]:
            amax[i] = v
        mag[i] -= amin[i]
        try:
            mag[i] /= (amax[i] - amin[i])
        except ZeroDivisionError:
            mag[i] = 0.0
        mag[i] -= 0.5
    # this does the math because it knows how strong the x and y magnetic fields are it uses tan to find the directoins in radians
    heading = math.atan2(mag[0], mag[1])
    # it turns it from radians to degrees
    if heading < 0:
        heading += 2 * math.pi
    deg = math.degrees(heading)
    # the heading is given to one decmile place
    return round(deg, 1)

# this averages the heading over the last 8 (random number) to stop outliers effecting the heading too much
def get_heading():
    h = get_heading_single()
    heading_history.append(h)
    
    # it removes the oldest heading
    if len(heading_history) > 8:
        heading_history.pop(0)
    # simple average
    return round(sum(heading_history) / len(heading_history), 1)


# Color sensor

def get_colour():
    # this was the old code which called for the color inside the main program but it was slowing it down too much
    """Ask Arduino for color. Expect "r,g,b\n" or similar.
    try:
        colourSensor.write(b"getcolour\n")
        response = colourSensor.readline().decode().strip()
        if not response:
            return None
        # support comma or space
        sep = "," if "," in response else " "
        parts = [p for p in response.replace(",", " ").split() if p.strip()]
        ints = []
        for p in parts[:3]:
            try:
                ints.append(int(p))
            except:
                ints.append(0)
        if len(ints) < 3:
            return None
        return ints
    except Exception as e:
        print("Colour read error:", e)
        return None"""
    # At the start of the prgram the file is empty and there might be an error with the communicatin
    try:
	# the code reads the vlaues from a file and gives back the RGB values
        with open("/tmp/colour.txt", "r") as f:
            line = f.read().strip()
            if "," in line:
                r, g, b = map(int, line.split(","))
                print(r, g, b)
                return [r, g, b]
    except:
        return None

# this takes the values and gives back the colour
def check_colours(vals):
    vals = get_colour()
    if vals is None:
        return None
    r, g, b = vals
    # these are the thresholds for checking what colour it is on 
    if r < BLACK_THRESHOLD and g < BLACK_THRESHOLD and b < BLACK_THRESHOLD:
        return "BLACK"
    if r > WHITE_THRESHOLD and g > WHITE_THRESHOLD and b > WHITE_THRESHOLD:
        return "WHITE"
    if r > g and r > b:
        return "RED"
    if g > r and g > b:
        return "GREEN"
    if b > r and b > g:
        return "BLUE"
    return "UNKNOWN"


# Navagation with the nodes, discovery, pathfinding

def squares_around(dists):
    # number of complete squares on each side of the robot 
    return [
        int(round((dists[FRONT_IDX] / SQUARE_SIZE) - 0.5)),
        int(round((dists[RIGHT_IDX] / SQUARE_SIZE) - 0.5)),
        int(round((dists[BACK_IDX] / SQUARE_SIZE) - 0.5)),
        int(round((dists[LEFT_IDX] / SQUARE_SIZE) - 0.5)),
    ]
# 
def discover_nodes(dists):
    # dists is the vlaues of all of the lidar
    """Populate connections for current_node based on squares around.
       Adds unexplored neighbor nodes that are not visited/blocked."""
    # connected is a list of dictonary of all nodes and what nodes they are connected to
    connected = []
    sq = squares_around(dists)
    # map relative indices to neighbor coordinates
    # facing is direction_facing, sensor i corresponds to (direction_facing + i) % 4 for i in [0,1,2,3] to get the facing
    for i in range(4):
        if sq[i] > 0:
            dir_idx = (direction_facing + i) % 4
            nx, ny = current_node[0], current_node[1]
            if dir_idx == 0:  # north
                ny += 1
            elif dir_idx == 1:  # east
                nx += 1
            elif dir_idx == 2:  # south
                ny -= 1
            else:  # west
                nx -= 1
            neighbor = [nx, ny]
	    # the only reason a square will not be added to connections is if it has prevoulsy been taged as a death square
            if tuple(neighbor) not in blocked:
		# Otherwise it is adds the connectoin to the list
                connected.append(tuple(neighbor))
		# it checks if the srounding squares have been visited, if not it adds it to a list of unexplored nodes
                if tuple(neighbor) not in visited and tuple(neighbor) not in unexplored_nodes:
                    unexplored_nodes.append(tuple(neighbor))
    connections[tuple(current_node)] = connected

# this choses the what node it is going to go to next
def get_next_node():
    # it gets the list of unexploded nodes and if   
    while unexplored_nodes:
        cand = unexplored_nodes.pop()
        if cand not in visited and cand not in blocked:
            return list(cand)
    return [0, 0]

def find_path_bfs(start, goal):
    """BFS in connections graph. returns list of node lists from start to goal inclusive or [] if not found."""
    from collections import deque
    if start == goal:
        return [start]
    q = deque()
    q.append([tuple(start)])
    seen = set([tuple(start)])
    while q:
        path = q.popleft()
        node = path[-1]
        neighs = connections.get(node, [])
        for n in neighs:
            if n in seen or n in blocked:
                continue
            newpath = list(path) + [n]
            if list(n) == goal:
                return [list(x) for x in newpath]
            seen.add(n)
            q.append(newpath)
    return []

# -------------------------
# MOVEMENT helpers
# -------------------------
def angle_diff(a, b):
    """Smallest signed difference a-b in degrees [-180,180]."""
    d = (a - b + 180) % 360 - 180
    return d

def turn_to_angle(target_angle, heading):
    """Turn toward target_angle. Return True when within tolerance."""
    diff = angle_diff(target_angle, heading)
    if abs(diff) < TURN_TOLERANCE_DEG:
        stop_all()
        return True
    # speed proportional to angle magnitude
    speed = int(min(TURN_MAX_SPEED, max(TURN_MIN_SPEED, (abs(diff) / 180.0) * TURN_MAX_SPEED)))
    if diff > 0:
        # need to increase heading -> turn CW or CCW depending on your servo orientation
        # Using same scheme as earlier code:
        # if diff > 0 (target > heading): turn CW by spinning motors opposite directions
        wheel(LEFT_MOTOR_ID, speed, "cw")
        wheel(RIGHT_MOTOR_ID, speed, "cw")
    else:
        wheel(LEFT_MOTOR_ID, speed, "ccw")
        wheel(RIGHT_MOTOR_ID, speed, "ccw")
    return False

# simple PD straightening based on left vs right lidar front sides
last_straight_error = 0.0
last_straight_time = None

def straighten_using_lidar(dists, heading):
    """Attempt to make robot aligned in the corridor / tile using left/right readings.
       Returns True when straight enough."""
    global last_straight_error, last_straight_time
    # pick sides: when moving forward compare front-left (index 7) and front-right (index 1)
    fl = dists[7]  # front-left diagonal (if available)
    fr = dists[1]  # front-right diagonal
    # fallback to LEFT_IDX and RIGHT_IDX if diagonals are bad
    if fl <= 0 or fr <= 0:
        fl = dists[LEFT_IDX]
        fr = dists[RIGHT_IDX]
    error = fr - fl  # positive -> robot is rotated clockwise (right is farther away)
    now = time.time()
    dt = (now - last_straight_time) if last_straight_time else 0.05
    de = (error - last_straight_error) / dt if dt > 0 else 0.0

    # PD output: generate small heading adjustment
    out = STRAIGHT_PD_P * error + STRAIGHT_PD_D * de
    last_straight_error = error
    last_straight_time = now

    if abs(error) < STRAIGHT_DIFF_THRESHOLD:
        stop_all()
        return True

    # compute target heading nudged by out (clamped)
    adj = max(-STRAIGHT_ANGLE_ADJUST, min(STRAIGHT_ANGLE_ADJUST, out / 10.0))
    target = (heading + adj) % 360
    turn_to_angle(target, heading)
    return False

def move_to_next_tile(dists, target_tile_offset):
    """
    Move forward/backward to reach the requested tile offset in front:
    target_tile_offset: number of tiles forward to move (1 means 1 tile ahead)
    Returns True when reached tile center.
    """
    # We will use front and back sensor (FRONT_IDX, BACK_IDX) to estimate centering
    front = dists[FRONT_IDX]
    back = dists[BACK_IDX]
    # compute expected front/back distances when centered on that tile:
    # If moving forward N tiles, the front sensor should read roughly (N - 0.5) * SQUARE_SIZE
    # and back sensor should read (total_ahead + total_back - ...). Simpler: aim to move until
    # front decreases to (SQUARE_SIZE * (target_tile_offset - 0.5))
    if target_tile_offset <= 0:
        return True

    desired_front = 9 # SQUARE_SIZE * (target_tile_offset - 0.5)
    # if the front reading is larger than desired, drive forward; if smaller, back up
    error = front - desired_front

    # stopping criteria: if front is within ~3 cm or change small
    if abs(error) < 3.0:
        stop_all()
        return True

    # basic proportional speed
    speed = int(max(MIN_SPEED, min(DEFAULT_MOTOR_SPEED, (abs(error) / (SQUARE_SIZE * 2.0)) * DEFAULT_MOTOR_SPEED)))
    if error > 0:
        # front is too far away -> move forward
        # differential correction: keep front-left/right similar
        # small correction to motors using left/right lateral sensor difference:
        lat_diff = (dists[7] if dists[7] > 0 else dists[LEFT_IDX]) - (dists[1] if dists[1] > 0 else dists[RIGHT_IDX])
        # apply small offset
        left_speed = speed - int(max(-50, min(50, lat_diff * 2)))
        right_speed = speed + int(max(-50, min(50, lat_diff * 2)))
        wheel(LEFT_MOTOR_ID, max(0, left_speed), "cw")
        wheel(RIGHT_MOTOR_ID, max(0, right_speed), "ccw")
    else:
        # we are too close -> back up
        lat_diff = (dists[7] if dists[7] > 0 else dists[LEFT_IDX]) - (dists[1] if dists[1] > 0 else dists[RIGHT_IDX])
        left_speed = speed - int(max(-50, min(50, lat_diff * 2)))
        right_speed = speed + int(max(-50, min(50, lat_diff * 2)))
        wheel(LEFT_MOTOR_ID, max(0, left_speed), "ccw")
        wheel(RIGHT_MOTOR_ID, max(0, right_speed), "cw")
    return False

# -------------------------
# MAIN LOOP
# -------------------------
try:
    last_lidar_read = 0
    while True:
        # button handling (toggle motors)
        button_state = GPIO.input(BUTTON_PIN)
        if button_state == GPIO.LOW and last_button_state == GPIO.HIGH:
            #if len(target_angles) != 4:
            #    target_angles.append(heading)
            #    print(target_angles)
            #else:
            motors_running = not motors_running
            time.sleep(0.15)
	    
		
        last_button_state = button_state

        heading = get_heading()
        print(heading)
        # lidar read
        if time.time() - last_lidar_read > LIDAR_READ_INTERVAL:
            distance = read_lidar()
            last_lidar_read = time.time()

            # piggyback colour read on LIDAR interval
            colours = get_colour()
            colour_name = check_colours(colours) if colours else None
            if colour_name:
                print("Color:", colour_name)

        if not motors_running:
            stop_all()
            continue

        # high level state machine
        print("STATE:", search_state, "current:", current_node, "visited:", len(visited))
        if search_state == SearchStates.DISCOVER_NODES:
            discover_nodes(distance)
            search_state = SearchStates.PICK_NEXT_NODE

        elif search_state == SearchStates.PICK_NEXT_NODE:
            # select next unexplored node
            target_node = get_next_node()
            print("target node:", target_node)
            # find path using BFS
            target_path = find_path_bfs(tuple(current_node), target_node)
            if not target_path or len(target_path) == 0:
                # nothing found -> go home
                target_path = [tuple(current_node), (0, 0)]
            # convert to list-of-lists
            target_path = [list(x) for x in target_path]
            path_idx = 1  # index of next node to go to (0 is current)
            search_state = SearchStates.TURN_TO_ANGLE

        elif search_state == SearchStates.TURN_TO_ANGLE:
            if path_idx >= len(target_path):
                # arrived or no where -> go discover again
                search_state = SearchStates.DISCOVER_NODES
                continue
            next_node = target_path[path_idx]
            # compute required direction relative to current_node
            dx = next_node[0] - current_node[0]
            dy = next_node[1] - current_node[1]
            angle = None
            if dx == 0 and dy > 0:
                angle = target_angles[0]  # north
                new_facing = 0
            elif dx == 0 and dy < 0:
                angle = target_angles[2]  # south
                new_facing = 2
            elif dy == 0 and dx > 0:
                angle = target_angles[1]  # east
                new_facing = 1
            elif dy == 0 and dx < 0:
                angle = target_angles[3]  # west
                new_facing = 3
            else:
                # unexpected: treat as done
                path_idx += 1
                continue
            print(angle)
            if turn_to_angle(angle % 360, heading):
                direction_facing = new_facing
                # measure how many squares ahead we are currently (for move goal)
                row_pos = squares_around(distance)[2]  # back sensing used earlier; keep existing logic
                # store tile offset as 1 tile forward (target) by default
                desired_tiles_forward = 1
                search_state = SearchStates.MOVE_NODE

        elif search_state == SearchStates.MOVE_NODE:
            # if color says black before moving into next tile, back off and mark blocked
            if colour_name == "BLACK":
                # compute neighbor coordinates that we're about to enter
                dx = target_path[path_idx][0] - current_node[0]
                dy = target_path[path_idx][1] - current_node[1]
                blocked_coord = (current_node[0] + dx, current_node[1] + dy)
                print("Black detected: marking blocked", blocked_coord)
                backwards()
                time.sleep(1)
                blocked.add(blocked_coord)
                # update connections to not include this neighbor
                conns = connections.get(tuple(current_node), [])
                connections[tuple(current_node)] = [c for c in conns if c != blocked_coord]
                # re-pick next node
                search_state = SearchStates.PICK_NEXT_NODE
                stop_all()
                continue

            # attempt to move forward one tile (target tile offset = 1)
            reached = move_to_next_tile(distance, 1)
            if reached:
                # update position
                current_node = target_path[path_idx]
                visited.add(tuple(current_node))
                print("Arrived at", current_node)
                # if reached goal node choose next behavior
                if current_node == target_node:
                    search_state = SearchStates.STRAIGHTEN
                else:
                    path_idx += 1
                    search_state = SearchStates.TURN_TO_ANGLE

        elif search_state == SearchStates.STRAIGHTEN:
            """done = straighten_using_lidar(distance, heading)
            if done:
                search_state = SearchStates.DISCOVER_NODES"""
            search_state = SearchStates.DISCOVER_NODES

        else:
            # unknown state -> restart discover
            search_state = SearchStates.DISCOVER_NODES

        time.sleep(0.02)

finally:
    stop_all()
    serial_connection.write_data(LEFT_MOTOR_ID, 24, [0])
    serial_connection.write_data(RIGHT_MOTOR_ID, 24, [0])
    serial_connection.close()
    try:
        lidar.close()
    except:
        pass
    try:
        colourSensor.close()
    except:
        pass
