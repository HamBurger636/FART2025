import serial
import time

# Open serial connection to the camera
ser = serial.Serial('/dev/serial0', 9600, timeout=1)
time.sleep(2)  # wait for camera to start up before reading


while True:
	# gets any data that is sent through by the camera
	data = ser.readline().decode().strip()
	# checks if there is any data sent through
	if data != None:
		# prints what it recived
		print("recived something: ", data)
		# if the camera sends a 0 it is nothing but it it sends a 1 there is something there
		# this file communicats to the main project by printing it into a file which is read by the main file
		if data == '1':
			# if it is something there it says 1
			print("blue or silver")
			with open("/tmp/AI_camera.txt", "w") as f:
				f.write('1')
		else:
			with open("/tmp/AI_camera.txt", "w") as f:
				f.write('0')

			

ser.close()




