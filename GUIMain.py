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

from PyQt5 import QtWidgets
from PyQt5.QtCore import QThread, QObject, pyqtSignal, Qt, pyqtSlot
from PyQt5.QtWidgets import QCompleter
from designerFile import Ui_MainWindow
import sys
from Balloon_Coordinates import Balloon_Coordinates
from satelliteTrackingMath import trackMath
from Ground_Station_Arduino import Ground_Station_Arduino
import serial.tools.list_ports
import time


# todo: make window scale/look good (bigger text)
# todo: display balloon info and calculation
# todo: display error messages in GUI
# todo: add large steps to manual controls
# todo: refresh arduino list/autoconnect to arduino?, check  if disconnected?
# todo: write documentation

# todo: automatically grab initial azimuth/elevation
# todo: incorporate IMU
# todo: predict next iridium ping
# todo: implement map of balloon location


class Window(QtWidgets.QMainWindow, Ui_MainWindow):
    # displayCalculationsSignal = pyqtSignal(float, float, float)
    # testSignal = pyqtSignal()

    def __init__(self):
        super(Window, self).__init__()

        self.setupUi(self)

        self.IMEIList = Balloon_Coordinates.list_IMEI()

        self.ports = serial.tools.list_ports.comports()
        self.portNames = []

        self.arduinoConnected = False
        self.IMEIAssigned = False
        self.calibrated = False

        self.tracking = False

        self.GSArduino = None  # classes will be instantiated later
        self.Balloon = None

        self.trackThread = None
        self.worker = None

        self.GSLat = 0
        self.GSLong = 0
        self.GSAlt = 0

        self.IMEIComboBox.addItem("")
        for i in range(len(self.IMEIList)):
            self.IMEIComboBox.addItem(self.IMEIList[i])

        for port, desc, hwid in sorted(self.ports):
            # self.COMPortComboBox.addItem("[{}] {}: {}".format(i, port, desc))
            self.COMPortComboBox.addItem(desc)
            self.portNames.append("{}".format(port))

        completer = QCompleter(self.IMEIList)
        completer.setFilterMode(Qt.MatchContains)
        self.IMEIComboBox.setEditable(True)
        self.IMEIComboBox.setCompleter(completer)

        self.confirmIMEIButton.clicked.connect(self.assignIMEI)

        self.GPSRequestButton.clicked.connect(self.getGSLocation)
        self.confirmGSLocationButton.clicked.connect(self.setGSLocation)

        self.calibrateButton.clicked.connect(self.calibrate)

        self.connectToArduinoButton.clicked.connect(self.connectToArduino)

        self.tiltUpButton.clicked.connect(self.tiltUp)
        self.tiltDownButton.clicked.connect(self.tiltDown)
        self.panLeftButton.clicked.connect(self.panLeft)
        self.panRightButton.clicked.connect(self.panRight)

        self.startButton.clicked.connect(self.checkIfReady)
        self.stopTrackingButton.clicked.connect(self.stopTracking)

    def assignIMEI(self):
        if self.IMEIComboBox.currentIndex() != 0:
            self.IMEIAssigned = True
            self.Balloon = Balloon_Coordinates(self.IMEIList[self.IMEIComboBox.currentIndex() - 1])
            self.Balloon.print_info()
            self.errorMessageBox.setPlainText("Balloon selected!")
        else:
            print("select a balloon ")
            self.errorMessageBox.setPlainText("Please select a balloon IMEI")
            self.IMEIAssigned = False
        return

    def connectToArduino(self):
        if not self.arduinoConnected and self.COMPortComboBox.currentIndex() != 0:
            self.GSArduino = Ground_Station_Arduino(self.portNames[self.COMPortComboBox.currentIndex() - 1], 9600)
            self.errorMessageBox.setPlainText("connected to arduino!")
            self.arduinoConnected = True
        else:
            print("failed to connect to arduino")
            self.errorMessageBox.setPlainText("failed to connect to arduino")
        return

    def tiltUp(self):
        if self.arduinoConnected:
            self.GSArduino.adjustTiltUp()
            print("adjusting tilt up 1 degree")
            self.errorMessageBox.setPlainText("adjusting tilt up 1 degree")
        else:
            print("Unable to connect to ground station motors")
            self.errorMessageBox.setPlainText("Not connected to ground station motors")

        return

    def tiltDown(self):
        if self.arduinoConnected:
            self.GSArduino.adjustTiltDown()
            print("adjusting tilt down 1 degree")
            self.errorMessageBox.setPlainText("adjusting tilt down 1 degree")
        else:
            print("Unable to connect to ground station motors")
            self.errorMessageBox.setPlainText("Not connected to ground station motors")

        return

    def panLeft(self):
        if self.arduinoConnected:
            self.GSArduino.adjustPanNegative()
            print("adjusting pan 1 degree negative (left)")
            self.errorMessageBox.setPlainText("adjusting pan 1 degree negative")
        else:
            print("Unable to connect to ground station motors")
            self.errorMessageBox.setPlainText("Not connected to ground station motors")

        return

    def panRight(self):
        if self.arduinoConnected:
            self.GSArduino.adjustPanPositive()
            print("adjusting pan 1 degree positive (right)")
            self.errorMessageBox.setPlainText("adjusting pan 1 degree positive")
        else:
            print("Unable to connect to ground station motors")
            self.errorMessageBox.setPlainText("Not connected to ground station motors")

        return

    def getGSLocation(self):
        if self.arduinoConnected:
            check = self.GSArduino.warm_start()
            if not check:  # if the coords cannot be retrieved, return
                print("failed to get GPS coords, please try again")
                self.errorMessageBox.setPlainText("failed to get GPS coords, please try again")
                return
            time.sleep(.25)

            GSCoords = self.GSArduino.req_GPS()
            self.GSArduino.print_GPS()
            self.GSLat = GSCoords[0]
            self.GSLong = GSCoords[1]
            self.GSAlt = GSCoords[2]

            self.GSLatBox.setPlainText(str(self.GSLat))
            self.GSLongBox.setPlainText(str(self.GSLong))
            self.GSAltBox.setPlainText(str(self.GSAlt))
        else:
            print("arduino not connected")
            self.errorMessageBox.setPlainText("arduino not connected")

        return

    def setGSLocation(self):
        try:
            latStr = self.GSLatBox.toPlainText()
            latStr = latStr.strip()
            self.GSLat = float(latStr)

            print(self.GSLat)

            longStr = self.GSLongBox.toPlainText()
            self.GSLong = float(longStr)
            print(self.GSLong)

            altStr = self.GSAltBox.toPlainText()
            self.GSAlt = float(altStr)
            print(self.GSAlt)

            self.errorMessageBox.setPlainText("Ground station location entered successfully!")
        except ValueError:
            print("numbers only for GPS location")
            self.errorMessageBox.setPlainText("invalid GPS location entered. Only enter numbers")

    def calibrate(self):
        if self.arduinoConnected:
            try:
                startingAzimuthStr = self.startingAzimuthBox.toPlainText()
                startingAzimuth = float(startingAzimuthStr)
                print(startingAzimuth)

                startingElevationStr = self.startingElevationBox.toPlainText()
                startingElevation = float(startingElevationStr)
                print(startingElevation)

                self.GSArduino.calibrate(startingAzimuth, startingElevation)
                self.calibrated = True
                self.errorMessageBox.setPlainText("successfully calibrated!")
            except ValueError:
                print("numbers only for initial azimuth and elevation")
                self.errorMessageBox.setPlainText("invalid input for initial azimuth and elevation")
        else:
            print("not connected to arduino")
            self.errorMessageBox.setPlainText("not connected to arduino")

        return

    def checkIfReady(self):
        if self.arduinoConnected:
            print("Com port assigned")
        else:
            print("Com port not assigned")
            self.errorMessageBox.setPlainText("Please connect arduino")

        if self.IMEIAssigned:
            print("IMEI assigned")
        else:
            print("IMEI not assigned")
            self.errorMessageBox.setPlainText("Please select arduino")

        if self.calibrated:
            print("Calibrated!")
        else:
            print("starting position not set")
            self.errorMessageBox.setPlainText("Please set staring azimuth and elevation")

        print("\n")

        if self.arduinoConnected and self.IMEIAssigned != 0 and self.calibrated:
            self.errorMessageBox.setPlainText("starting tracking!")
            self.track()
            return True
        else:
            return False

    def track(self):
        self.tracking = True
        self.trackThread = QThread()
        self.worker = Worker()

        self.worker.moveToThread(self.trackThread)

        self.trackThread.started.connect(self.worker.track)

        self.worker.finished.connect(self.trackThread.quit)  # pycharm has bug, this is correct
        self.worker.finished.connect(self.worker.deleteLater)  # https://youtrack.jetbrains.com/issue/PY-24183?_ga=2.240219907.1479555738.1625151876-2014881275.1622661488
        self.trackThread.finished.connect(self.trackThread.deleteLater)

        # self.trackThread.output[float, float, float].connect(self.displayCalculations)

        self.startButton.setEnabled(False)

        self.trackThread.start()

    def stopTracking(self):
        self.tracking = False
        self.startButton.setEnabled(True)
        self.errorMessageBox.setPlainText("tracking stopped")
        return

    def displayCalculations(self, distance, azimuth, elevation):
        print("calling display calculations")
        self.distanceDisplay.setPlainText(str(distance))
        self.azimuthDisplay.setPlainText(str(azimuth))
        self.elevationDisplay.setPlainText(str(elevation))
        return


class Worker(QObject):
    finished = pyqtSignal()

    calcSignal = pyqtSignal(float, float, float)

    i = 0

    def track(self):
        timer = time.time() - 4
        while MainWindow.tracking:
            if (time.time() - timer) > 5:
                timer = time.time()
                Balloon_Coor = MainWindow.Balloon.get_coor_alt()

                # note that trackMath takes arguments as long, lat, altitude
                Tracking_Calc = trackMath(MainWindow.GSLong, MainWindow.GSLat, MainWindow.GSAlt, Balloon_Coor[1],
                                          Balloon_Coor[0], Balloon_Coor[2])

                distance = Tracking_Calc.distance
                newElevation = Tracking_Calc.elevation()
                newAzimuth = Tracking_Calc.azimuth()

                print(str(self.i) + " Distance " + str(distance) + " Azimuth: " + str(newAzimuth) + ", Elevation: " + str(newElevation))

                # MainWindow.distanceDisplay.setPlainText(str(distance))
                # MainWindow.azimuthDisplay.setPlainText(str(newAzimuth))
                # MainWindow.elevationDisplay.setPlainText(str(newElevation))

                # MainWindow.displayCalculations(distance, newAzimuth, newElevation)

                self.calcSignal.connect(MainWindow.displayCalculations)  # this seems to happen a lot for some reason
                self.calcSignal.emit(distance, newAzimuth, newElevation)

                MainWindow.GSArduino.move_position(newAzimuth, newElevation)

                self.i += 1

        print("All done!")
        # MainWindow.errorMessageBox.setPlainText("all done tracking!")
        self.finished.emit()  # same pycharm bug as above
        return


if __name__ == "__main__":

    app = QtWidgets.QApplication(sys.argv)
    MainWindow = Window()
    MainWindow.show()

    sys.exit(app.exec_())
