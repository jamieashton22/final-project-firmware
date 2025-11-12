import numpy as np
import serial
import serial.tools.list_ports
import sys
import time

ports = serial.tools.list_ports.comports()
serialInst = serial.Serial()
port = ''
portList = []

print("\n List of available ports \n ")
for onePort in ports:
    portList.append(str(onePort.device))
    print(str(onePort))

port = input("\n select port e.g '/dev/.....' : \n")

if port not in portList:
    print("Invalid port chosen")
    sys.exit(1)

print ("\n Port selected: ")
print(port)

chosenBaud = 9600  # SELECT BAUDRATE HERE 

ser = serial.Serial(port, chosenBaud )
time.sleep(2)

while True:

    command = input(">> ").strip()            #WRITING A COMMAND
    ser.write((command + '\n').encode())

    time.sleep(0.1)                             # TWEAK 
    while ser.in_waiting:
        response = ser.readline().decode().strip()
        if response:
            print("[Arduino] " + response)