from IMU import IMU
from satelliteTrackingMath import trackMath
from Ground_Station_Arduino import Ground_Station_Arduino
import serial.tools.list_ports
import time


def calcs():
    ports = serial.tools.list_ports.comports()
    portNames = []
    index = 0

    for port, desc, hwid in sorted(ports):
        print("[{}] {}: {}".format(index, port, desc))
        portNames.append("{}".format(port))
        index += 1

    user_port = int(input("Choose GS Port: "))
    print("{}".format(portNames[user_port]))

    GSMotors = Ground_Station_Arduino(portNames[user_port], 9600)
    IMUPort = IMU(portNames[user_port + 1], 115200)

    while True:
        newAzimuth = float(input("Enter azimuth to go to here: "))
        newElevation = float(input("Enter elevation to go to here: "))

        currLoc = IMUPort.readData()
        print(currLoc)
        GSMotors.calibrate(float(currLoc[0]), float(currLoc[1]))

        GSMotors.move_position(newAzimuth, newElevation)

        time.sleep(1)


calcs()
