from Balloon_Coordinates import Balloon_Coordinates
from Ground_Station_Coordinates import Ground_Station_Coordinates
from satelliteTrackingMath import trackMath
from Ground_Station_Motors import Ground_Station_Motors
import serial.tools.list_ports
import time
import csv

ports = serial.tools.list_ports.comports()
portNames = []

index = 0
for port, desc, hwid in sorted(ports):
        print("[{}] {}: {}".format(index, port, desc))
        portNames.append("{}".format(port))
        index += 1

user_port = int(input("Choose Port: "))
print("{}".format(portNames[user_port]))

GSMotors = Ground_Station_Motors(portNames[user_port], 9600)

timer = time.time()

oldAzimuth = float(input("Enter initial azimuth: "))
oldElevation = float(input("Enter initial elevation: "))

GSMotors.calibrate(oldAzimuth, oldElevation)

while True:
    if ((time.time() - timer) > 1):
        timer = time.time()

        newAzimuth = float(input("Enter new azimuth: "))
        newElevation = float(input("Enter new elevation: "))

        GSMotors.move_position(newAzimuth, newElevation)
