#!/usr/bin/env python3

import serial
import time

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1.0)
time.sleep(3) # 3 seconds
ser.reset_input_buffer() # to start with a fresh buffer
print("serial ok, received in pi")

try:
    while True:
        time.sleep(1) # 10ms
        # to read from arduino
        # if ser.in_waiting > 0:
        #     line = ser.readline().decode('utf-8').rstrip() # rstrip is to remove newline character
        #     print(line)

        # to write to arduino
        print("sending the message to arduino")
        ser.write("hello from the rpi\n".encode('utf-8'))
        while ser.in_waiting <= 0:
            time.sleep(0.01)
        response = ser.readline().decode('utf-8').rstrip()
        print(response)

except KeyboardInterrupt:
    print("close serial communication")
    ser.close() # always remember to close 