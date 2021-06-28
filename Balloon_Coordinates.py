"""
-------------------------------------------------------------------------------
MIT License
Copyright (c) 2021 Ronnel Walton
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

import requests;
import time;


class Balloon_Coordinates:
    BOREALIS_EPOCH = 1357023600;

    def __init__(self, imei):

        self.imei = imei;

        # Grab and Define IMEI's Latest Flight
        req = requests.get("https://borealis.rci.montana.edu/meta/flights?imei={}".format(self.imei));
        self.latest_flight = req.json()[-1];

        # Define UID
        flightTime = int(time.mktime(time.strptime(self.latest_flight, "%Y-%m-%d")) - 21600 - Balloon_Coordinates.BOREALIS_EPOCH);
        self.uid = (int(flightTime) << 24)|int(self.imei[8:]);
        return;

    
    @staticmethod
    def list_IMEI():
        # Request IMEI List
        req = requests.get('https://borealis.rci.montana.edu/meta/imeis');
        data = req.json();
        IMEIs = [];
        
        for imei in data:
            IMEIs.append(imei);
        return IMEIs;

    def get_coor_alt(self):
        req = requests.get("https://borealis.rci.montana.edu/flight?uid={}".format(self.uid));
        data = req.json();
        # Lat, Long, Alt
        self.coor_alt = [data['data'][-1][3], data['data'][-1][4], data['data'][-1][5]];
        return self.coor_alt;

    def print_info(self):
        self.get_coor_alt();
        print("IMEI: ", self.imei);
        print("Date:", self.latest_flight);
        print("Coordinates: (", self.coor_alt[0], ", " ,self.coor_alt[1], ")");
        print("Altitude: ", self.coor_alt[2]);
        return;

    pass




