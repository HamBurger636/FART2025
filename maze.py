from FART import FART, LidarDir, Directions
from enum import Enum
from math import copysign
from time import sleep


SQUARE_SIZE = 28
LIDAR_RING_DIAMETER = 5

MIN_SPEED = 20
MAX_SPEED = 100
MAX_DISTANCE = 20
MIN_DISTANCE = 0

TURN_SPEED_MAX = 80
TURN_SPEED_MIN = 20
TURN_DISTANCE_MAX = 180
TURN_DISTANCE_MIN = 5







class SearchStates(Enum):
    SEARCH_NODE = 0
    DISCOVER_NODES = 1
    PICK_NEXT_NODE = 2,
    MOVE_NODE = 3

def turn_speed(distance: float):
    if distance > TURN_DISTANCE_MAX:
        return TURN_SPEED_MAX

    # get slower as you get closer
    return max(distance - TURN_DISTANCE_MIN, 0) / (TURN_DISTANCE_MAX - TURN_DISTANCE_MIN) * (TURN_SPEED_MIN - TURN_SPEED_MAX) + TURN_SPEED_MIN


def drive_speed(distance: float):
    if distance > MAX_DISTANCE:
        return MAX_SPEED

    # get slower as you get closer to goal
    return max(distance-MIN_DISTANCE, 0)/(MAX_DISTANCE-MIN_DISTANCE) * (MAX_SPEED-MIN_SPEED) + MIN_SPEED


def turn_to(fart: FART, dir: Directions):
    target_heading = (NORTH_HEADING + 90 * dir.value) % 360
    heading = fart.get_heading()

    diff = (heading % 360) - (target_heading % 360)
    diff = diff if abs(diff) <= 180 else diff - copysign(360, diff)


def number_of_squares(fart: FART):
    front = fart.get_lidar(LidarDir.FRONT)
    back = fart.get_lidar(LidarDir.BACK)

    return (front + back + LIDAR_RING_DIAMETER) // SQUARE_SIZE


def move_to_square(fart: FART, square_number: int):
    # get values
    front = fart.get_lidar(LidarDir.FRONT)
    back = fart.get_lidar(LidarDir.BACK)
    # calculate distances
    target_b = SQUARE_SIZE * (square_number + 0.5)
    target_f = number_of_squares(fart) * SQUARE_SIZE - target_b
    # calculate how far front + back are, swapping first item to keep sign
    # consistent and then averaging
    diff_front = front - target_f
    diff_back = target_b - back
    avg_diff = (diff_front + diff_back) / 2

    if (abs(diff_front) < 3 and abs(diff_back) < 3) or \
            (avg_diff < 5 and abs(diff_front) < 10 and abs(diff_back) < 10):
        fart.stop()
        return True
    elif copysign(1, diff_front) == copysign(diff_back, 1):
        fart.forward(copysign(avg_diff, drive_speed(abs(avg_diff))))
    else:
        dist = diff_back if abs(diff_back) < abs(diff_front) else diff_front
        fart.forward(copysign(dist, drive_speed(abs(dist))))

    return False


def lidar_to_dir(lidar_dir: LidarDir):
    if lidar_dir == LidarDir.FRONT:
        return facing
    if lidar_dir == LidarDir.LEFT:
        return Directions((facing.value - 1) % 4)
    if lidar_dir == LidarDir.RIGHT:
        return Directions((facing.value + 1) % 4)

    raise Exception("INVALID lidar Direction")


def node_in_direction(dir: Directions):
    x, y = current_node
    return [x if dir.value % 2 == 0 else x - dir.value + 2, y if dir.value % 2 == 1 else 1 - dir.value + 1]


def discover_nodes(fart: FART, first_node: bool = False):
    connected = []
    # Check all directions except back unless it's the first node
    for direction in list(LidarDir) if first_node else [LidarDir.RIGHT, LidarDir.FRONT, LidarDir.LEFT]:
        # only consider directions that are open
        if fart.get_lidar(direction) > SQUARE_SIZE:
            new_node = node_in_direction(lidar_to_dir(direction))
            # add open node to connected nodes (valid movement)
            connected.append(new_node)
            # only add to unexplored if it hasn't been visited
            if new_node not in visited:
                unexplored_nodes.append(new_node)

    connections[current_node] = connected


def next_node():
    # if there are no unexplored nodes return home
    if len(unexplored_nodes) == 0:
        return [0, 0]
    # get the next unexplored node
    next_node = unexplored_nodes.pop()
    # make sure the node hasn't been visited yet
    while next_node in visited:
        if len(unexplored_nodes) == 0:
            return [0, 0]
        next_node = unexplored_nodes.pop()

    return next_node


# Main Program
with FART(motor_port="/dev/ttyACM0", motor_baud_rate=115200, motor_left=0, motor_right=1, lidar_port="/dev/ttyACM1", lidar_offset=[3.1,1.7,1.8,2.4,1,2.4,1.8,2.3]) as Fart: 
    # initialise variables
    unexplored_nodes = []
    connections = {}
    visited = set()
    current_node = [0, 0]
    facing = Directions.NORTH

    #  Do initial readings
    NORTH_HEADING = Fart.get_heading()
    discover_nodes(Fart, True)
    search_state = SearchStates.SEARCH_NODE

    # High level loop
    while True:
		Fart.read_lidar()
        if search_state == SearchStates.SEARCH_NODE:
            pass
        elif search_state == SearchStates.DISCOVER_NODES:
            pass
        elif search_state == SearchStates.PICK_NEXT_NODE:
            pass
        elif search_state == SearchStates.MOVE_NODE:
            pass
        else:
            break

