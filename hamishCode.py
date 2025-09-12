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

# Enable torque
serial_connection.write_data(left_motor_id, 24, [1])
serial_connection.write_data(right_motor_id, 24, [1])

# IMU setup
imu = ICM20948()

# Axis indices
X, Y, Z = 0, 1, 2
AXES = X, Y

# Function to control a wheel
def wheel(motor_id, speed, direction="cw"):
    if direction == "cw":
        value = speed  # 0–1023
    else:
        value = 1024 + speed  # 1024–2047 means CCW
    serial_connection.write_data(motor_id, 32, [value & 0xFF, (value >> 8) & 0xFF])

# Stop motor
def stop_motor(motor_id):
    serial_connection.write_data(motor_id, 32, [0, 0])

def forwards():
	wheel(left_motor_id, 300, "cw")
    wheel(right_motor_id, 300, "ccw")

def backwards():
	wheel(left_motor_id, 300, "ccw")
    wheel(right_motor_id, 300, "cw")

# Turn towards a target angle
def turn(target_angle, heading):
    diff = (target_angle - heading + 360) % 360  
    if diff > 180:
        # Turn CCW
        wheel(left_motor_id, 300, "ccw")
        wheel(right_motor_id, 300, "ccw")
    else:
        # Turn CW
        wheel(left_motor_id, 300, "cw")
        wheel(right_motor_id, 300, "cw")

target_angle = 90  

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
	return distances 

try:
    while True:
        button_state = GPIO.input(BUTTON_PIN)

        if button_state == GPIO.LOW and last_button_state == GPIO.HIGH:
            motors_running = not motors_running
        
        mag = list(imu.read_magnetometer_data())
        heading = math.atan2(mag[AXES[0]], mag[AXES[1]])
        if heading < 0:
            heading += 2 * math.pi
        heading = math.degrees(heading)
        heading = round(heading)
        print(f"Heading: {heading}°")
		
        if time.time() - last_read > LIDAR_READ_INTERVAL:
            distance = read_lidar()
            print(distance)
            print("Number of tiles ahead:/t", distance[0]/28)
            last_read = time.time()
		
        if motors_running:
			"""
			# Turn until heading is within 10 degrees of target
            if abs(target_angle - heading) > 10:
                turn(target_angle, heading)
            else:
                stop_motor(left_motor_id)
                stop_motor(right_motor_id)"""
            if distance[0] < 15:
				if distance[2] > 15:
					target_angle = heading + 90
					if abs(target_angle - heading) > 10:
						turn(target_angle, heading)
				elif distance[6] < 15:
					target_angle = heading - 90
					if abs(target_angle - heading) > 10:
						turn(target_angle, heading)
				else:
					while distance[2] < 15 and distance[6] < 15:
						 if distance[2] > 15:
							target_angle = heading + 90
							if abs(target_angle - heading) > 10:
								turn(target_angle, heading)
						elif distance[6] < 15:
							target_angle = heading - 90
							if abs(target_angle - heading) > 10:
								turn(target_angle, heading)
			else:
				forward()
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

