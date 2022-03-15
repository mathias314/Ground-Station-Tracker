"""
-------------------------------------------------------------------------------
MIT License
Copyright (c) 2021 Mathew Clutter
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


class IMU:

    def __init__(self, COM_Port, baudrate):
        self.Port_Name = COM_Port
        self.Baudrate = baudrate
        self.COM_Port = serial.Serial(port=self.Port_Name, baudrate=self.Baudrate, timeout=.1)
        self.Coordinates = []
        self.attempt_num = 0
        return

    def requestData(self):
        # get the current elevation and azimuth from imu
        message = "R"
        self.COM_Port.write(bytes(message, "utf-8"))
        time.sleep(.05)

        serialData = self.COM_Port.readline()
        decodedData = serialData.decode('ascii')

        print(decodedData)

        currPos = decodedData.split(',')

        print(currPos)

        return currPos
