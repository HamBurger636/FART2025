#!/usr/bin/env python3
# maze_nav.py
# Improved maze navigation: black-square avoidance, pathfinding, straightening, and fixes.

import time
import math
from enum import Enum
from random import randint
import subprocess
import atexit
import os

# hardware libs (keep your existing setup)
from pyax12.connection import Connection
from icm20948 import ICM20948
import RPi.GPIO as GPIO
import serial


# CONFIG / TUNABLES


# This is for the lidar where it defins the communication protcals and the 
LIDAR_PORT = "/dev/ttyACM1"
LIDAR_BAUD = 115200
LIDAR_READ_INTERVAL = 0.25

LIDAR_OFFSET = [3.1, 1.7, 1.8, 2.4, 1.0, 2.4, 1.8, 2.3]  # per-sensor offsets because they are all conststantally off by a set amount 
SQUARE_SIZE = 28.0  # cm (tile size)
FRONT_IDX = 0
RIGHT_IDX = 2
BACK_IDX = 4
LEFT_IDX = 6

BUTTON_PIN = 26

# servos
SERVO_PORT = "/dev/ttyACM0"
SERVO_BAUD = 1000000
LEFT_MOTOR_ID = 15
RIGHT_MOTOR_ID = 18
DEFAULT_MOTOR_SPEED = 480
MIN_SPEED = 300

# distances
MAX_DISTANCE = 200  # cm (use a generous bound)
MIN_DISTANCE = 0

# turning PID-ish params
TURN_TOLERANCE_DEG = 5
TURN_MAX_SPEED = 480
TURN_MIN_SPEED = 300

# straightening thresholds
STRAIGHT_DIFF_THRESHOLD = 3.0  # cm difference left vs right considered "not straight"
STRAIGHT_ANGLE_ADJUST = 6.0  # degrees to nudge by during straightening
STRAIGHT_PD_P = 6.0
STRAIGHT_PD_D = 2.0

# color thresholds
BLACK_THRESHOLD = 10
WHITE_THRESHOLD = 30

# STATE
GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

motors_running = False
last_button_state = GPIO.input(BUTTON_PIN)

# servo connection
serial_connection = Connection(port=SERVO_PORT, baudrate=SERVO_BAUD)

# enable torque
serial_connection.write_data(LEFT_MOTOR_ID, 24, [1])
serial_connection.write_data(RIGHT_MOTOR_ID, 24, [1])

# IMU
imu = ICM20948()

# LIDAR serial
lidar = serial.Serial(LIDAR_PORT, LIDAR_BAUD, timeout=0.5)
lidar.flushInput()
lidar.flushOutput()
last_lidar_read = 0
distance = [0.0] * 8

# color sensor (arduino) - expects reply like "r,g,b\n" when sending "getcolour\n"
colourSensor = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.5)
# this is for opening the other program which checks for the colours on the ground becasue when it was in the main program it was slowing it down a lot
# Path to the helper program
COLOUR_HELPER_PATH = "/home/fart/fart/colorsensor.py"  # adjust path

# Start the colour helper in the background
colour_proc = subprocess.Popen(
    ["python3", COLOUR_HELPER_PATH],
    stdout=subprocess.DEVNULL,
    stderr=subprocess.DEVNULL
)



# navigation state
class SearchStates(Enum):
    SEARCH_NODE = 0
    DISCOVER_NODES = 1
    PICK_NEXT_NODE = 2
    TURN_TO_ANGLE = 3
    MOVE_NODE = 4
    STRAIGHTEN = 5

direction_facing = 0  # 0: north (forward), 1: east, 2: south, 3: west
current_node = [0, 0]  # x,y
visited = set([tuple(current_node)])
blocked = set()  # coordinates considered blocked (black)
unexplored_nodes = []
connections = {}  # adjacency: tuple(node) -> list of neighbor tuples
search_state = SearchStates.DISCOVER_NODES
target_node = [0, 0]
target_path = []
path_idx = 0

# target angles for facing directions (calibrate these to your robot), i am doing this instead of just adding or removing 90 from the current facing becasue there will be a slow drift in direction over many turns
target_angles = [348, 75, 155, 244]  # indexes: 0:north,1:east,2:south,3:west


# -------------------------
# HELPER: servos / motors
# -------------------------
def wheel(motor_id, speed, direction="cw"):
    # Write to AX12 style motor: speed range 0-1023; for CCW add 1024
    speed = max(0, min(1023, int(speed)))
    if direction == "cw":
        value = speed
    else:
        value = 1024 + speed
    serial_connection.write_data(motor_id, 32, [value & 0xFF, (value >> 8) & 0xFF])

def stop_motor(motor_id):
    serial_connection.write_data(motor_id, 32, [0, 0])

def forwards(speed=None):
    s = DEFAULT_MOTOR_SPEED if speed is None else int(speed)
    wheel(LEFT_MOTOR_ID, s, "cw")
    wheel(RIGHT_MOTOR_ID, s, "ccw")

def backwards(speed=None):
    s = DEFAULT_MOTOR_SPEED if speed is None else int(speed)
    wheel(LEFT_MOTOR_ID, s, "ccw")
    wheel(RIGHT_MOTOR_ID, s, "cw")

def stop_all():
    stop_motor(LEFT_MOTOR_ID)
    stop_motor(RIGHT_MOTOR_ID)

# -------------------------
# LIDAR & IMU utilities
# -------------------------
def read_lidar():
    """Read 20 bytes frame as original code; parse safely and return list of 8 distances (cm)."""
    global lidar
    try:
        lidar.flushInput()
        resp = lidar.read(20)
        if len(resp) < 20:
            # partial read: return previous distances unchanged
            return distance
        # parse 8 distances from bytes 2..17 as big-endian 2-byte values then /10 then offset
        d = []
        for i in range(8):
            idx = 2 + i*2
            raw = (resp[idx] << 8) + resp[idx+1]
            cm = raw / 10.0 - LIDAR_OFFSET[i]
            d.append(round(max(0.0, cm), 2))
        return d
    except Exception as e:
        print("LIDAR read error:", e)
        return distance

# IMU heading with simple min/max calibration
amin = [-71.40, -30.45, -89.25]
amax = [24.45, 70.05, 13.65]
heading_history = []

def get_heading_single():
    """Return instantaneous heading from magnetometer (0-360)."""
    mag = list(imu.read_magnetometer_data())
    # calibrate inline (keep global amin/amax updated)
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

    heading = math.atan2(mag[0], mag[1])
    if heading < 0:
        heading += 2 * math.pi
    deg = math.degrees(heading)
    return round(deg, 1)

def get_heading():
    """Smoothed heading (moving average)."""
    h = get_heading_single()
    heading_history.append(h)
    if len(heading_history) > 8:
        heading_history.pop(0)
    # simple average
    return round(sum(heading_history) / len(heading_history), 1)

# -------------------------
# Color sensor
# -------------------------
def get_colour():
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
    try:
        with open("/tmp/colour.txt", "r") as f:
            line = f.read().strip()
            if "," in line:
                r, g, b = map(int, line.split(","))
                print(r, g, b)
                return [r, g, b]
    except:
        return None

def check_colours(vals):
    if vals is None:
        return None
    r, g, b = vals
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

# -------------------------
# NAV: nodes, discovery, pathfinding
# -------------------------
def squares_around(dists):
    """Return the number of full squares available in front,right,back,left from LIDAR distances."""
    # number of complete squares ahead on each side (rounded down)
    return [
        int(round((dists[FRONT_IDX] / SQUARE_SIZE) - 0.5)),
        int(round((dists[RIGHT_IDX] / SQUARE_SIZE) - 0.5)),
        int(round((dists[BACK_IDX] / SQUARE_SIZE) - 0.5)),
        int(round((dists[LEFT_IDX] / SQUARE_SIZE) - 0.5)),
    ]

def discover_nodes(dists):
    """Populate connections for current_node based on squares around.
       Adds unexplored neighbor nodes that are not visited/blocked."""
    connected = []
    sq = squares_around(dists)
    # map relative indices to neighbor coordinates
    # facing is direction_facing; sensor i corresponds to (direction_facing + i) % 4 for i in [0,1,2,3]
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
            if tuple(neighbor) not in blocked:
                connected.append(tuple(neighbor))
                if tuple(neighbor) not in visited and tuple(neighbor) not in unexplored_nodes:
                    unexplored_nodes.append(tuple(neighbor))
    connections[tuple(current_node)] = connected

def get_next_node():
    """Pop next unexplored node (LIFO). If none, return home (0,0)."""
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

    desired_front = 9# SQUARE_SIZE * (target_tile_offset - 0.5)
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
