import serial
import time

# Open serial connection (change port if needed)
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
time.sleep(2)  # wait for Arduino to reset after opening serial


while True:

    # Wait for Arduino reply
    response = ser.readline().decode().strip()

    # If the response is comma- or space-separated, split and print nicely
    if response:
        with open("/tmp/colour.txt", "w") as f:
            f.write(response)
            print(response)

ser.close()
