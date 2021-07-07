"""
-------------------------------------------------------------------------------
MIT License
Copyright (c) 2021 Mathew Clutter and Ronnel Walton
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
-------------------------------------------------------------------------------
"""

import serial
import serial.tools.list_ports
import time


class Ground_Station_Arduino:

    def __init__(self, COM_Port, baudrate):
        self.Port_Name = COM_Port
        self.Baudrate = baudrate
        self.COM_Port = serial.Serial(port=self.Port_Name, baudrate=self.Baudrate, timeout=.1)
        self.Coordinates = []
        self.attempt_num = 0
        return

    def move_position(self, azimuth, elevation):  # takes in azimuth and elevation to move to
        pos = "M" + str(azimuth) + "," + str(elevation)
        print(pos)
        self.COM_Port.write(bytes(pos, "utf-8"))
        time.sleep(.05)
        return

    def adjustTiltUp(self):
        self.COM_Port.write(b'W')
        print("adjusting tilt up yeehaw")
        time.sleep(.05)
        return

    def adjustTiltDown(self):
        self.COM_Port.write(b'S')
        time.sleep(.05)
        return

    def adjustPanPositive(self):
        self.COM_Port.write(b'A')
        time.sleep(.05)
        return

    def adjustPanNegative(self):
        self.COM_Port.write(b'D')
        time.sleep(.05)
        return

    def calibrate(self, startingAzimuth, startingElevation):
        startPos = "C" + str(startingAzimuth) + "," + str(startingElevation)
        self.COM_Port.write(bytes(startPos, "utf-8"))
        time.sleep(.05)
        return

    def warm_start(self):
        self.Coordinates = self.req_GPS()
        return self.Coordinates

    def req_GPS(self):
        if self.attempt_num < 100:
            self.attempt_num += 1
            self.COM_Port.write(b'G')
            time.sleep(0.05)
            serial_data = self.COM_Port.readline()
            if len(serial_data) > 2:
                decoded_data = serial_data[:-2].decode('ascii')
                if len(decoded_data.split(",")) == 3:
                    self.Coordinates = []
                    Temp_Coor = decoded_data.split(",")
                    # S and W negative
                    for index in range(2):
                        self.Coordinates.append(float(Temp_Coor[index][:-1]))
                    self.Coordinates.append(float(Temp_Coor[2]))
                    # print(self.attempt_num, " Attempts")
                    self.attempt_num = 0
                else:
                    self.req_GPS()
            else:
                self.req_GPS()
        else:
            print("Failed to request GPS")
            # exit(1)
        return self.Coordinates

    def print_GPS(self):
        self.Coordinates = self.req_GPS()
        if len(self.Coordinates) > 0:
            print("[{},{},{}]".format(self.Coordinates[0], self.Coordinates[1], self.Coordinates[2]))
        return
