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


class IMU:

    def __init__(self, COM_Port, baudrate):
        self.Port_Name = COM_Port
        self.Baudrate = baudrate
        self.COM_Port = serial.Serial(port=self.Port_Name, baudrate=self.Baudrate, timeout=.1)

        self.declination = -11.53333

        return

    def readData(self):
        # get the current elevation and azimuth from imu

        self.COM_Port.flushInput()

        decodedData = ""

        currPos = []

        while len(currPos) < 2:

            serialData = self.COM_Port.readline()
            try:
                decodedData = serialData.decode('ascii')
            except: # if error decoding serial data try again
                decodedData = ""

            # print(decodedData)
            if decodedData:
                currPos = decodedData.split(',')

        convertedCurrPos = [float(currPos[0]) + self.declination, float(currPos[1])]  # need to adjust to true north

        print(convertedCurrPos)

        return convertedCurrPos
