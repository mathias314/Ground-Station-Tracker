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
from PyQt5.QtWidgets import QCompleter, QApplication, QDesktopWidget
from designerFile import Ui_MainWindow
import sys
from Balloon_Coordinates import Balloon_Coordinates
from satelliteTrackingMath import trackMath
from Ground_Station_Arduino import Ground_Station_Arduino
import serial.tools.list_ports
import time
from pylab import *
from sunposition import sunpos
from datetime import datetime
import csv
import statistics
import numpy as np

# todo: incorporate IMU?
# todo: predict next iridium ping!
# todo: calibrate without relying on the sun

DEBUG = True


class Window(QtWidgets.QMainWindow, Ui_MainWindow):
    def __init__(self):
        super(Window, self).__init__()

        self.setupUi(self)

        self.IMEIList = Balloon_Coordinates.list_IMEI()

        self.arduinoConnected = False
        self.IMEIAssigned = False
        self.GSLocationSet = False
        self.calibrated = False

        self.tracking = False

        self.GSArduino = None  # classes will be instantiated later
        self.Balloon = None

        self.trackThread = None
        self.worker = None

        self.GSLat = 0
        self.GSLong = 0
        self.GSAlt = 0

        self.startingAzimuth = 0
        self.startingElevation = 0

        self.IMEIComboBox.addItem("")
        for i in range(len(self.IMEIList)):
            self.IMEIComboBox.addItem(self.IMEIList[i])

        completer = QCompleter(self.IMEIList)
        completer.setFilterMode(Qt.MatchContains)
        self.IMEIComboBox.setEditable(True)
        self.IMEIComboBox.setCompleter(completer)

        self.ports = None
        self.portNames = []
        self.comPortCounter = 0
        self.refreshArduinoList()

        self.confirmIMEIButton.clicked.connect(self.assignIMEI)

        self.GPSRequestButton.clicked.connect(self.getGSLocation)
        self.confirmGSLocationButton.clicked.connect(self.setGSLocation)

        self.calibrateButton.clicked.connect(self.calibrate)

        self.refreshCOMPortsButton.clicked.connect(self.refreshArduinoList)
        self.connectToArduinoButton.clicked.connect(self.connectToArduino)

        self.degreesPerClickBox.setCurrentIndex(1)
        self.COMPortComboBox.setCurrentIndex(self.comPortCounter - 1)
        self.tiltUpButton.clicked.connect(self.tiltUp)
        self.tiltDownButton.clicked.connect(self.tiltDown)
        self.panCounterClockwiseButton.clicked.connect(self.panClockwise)
        self.panClockwiseButton.clicked.connect(self.panCounterClockwise)

        self.calculateStartingPosButton.clicked.connect(self.getStartingPos)

        self.startButton.clicked.connect(self.checkIfReady)
        self.stopButton.clicked.connect(self.stopTracking)

        self.predictionStartButton.clicked.connect(self.setPredictTrack)

        font = self.font()
        font.setPointSize(11)  # can adjust for sizing
        QApplication.instance().setFont(font)


        # self.showMaximized()
        # self.showFullScreen()

        self.predictingTrack = False

    def assignIMEI(self):
        if self.IMEIComboBox.currentIndex() != 0:  # SHOULD BE != FOR BOREALIS WEBSITE!
            self.IMEIAssigned = True
            print(self.IMEIComboBox.currentText())
            self.Balloon = Balloon_Coordinates(self.IMEIComboBox.currentText())
            testStr = self.Balloon.print_info()
            self.errorMessageBox.setPlainText(testStr)
            self.Balloon.getTimeDiff()
        else:
            print("select a balloon ")
            self.errorMessageBox.setPlainText("Please select a balloon IMEI")
            self.IMEIAssigned = False
        return

    def refreshArduinoList(self):
        self.COMPortComboBox.clear()
        self.ports = serial.tools.list_ports.comports()
        self.portNames = []
        self.comPortCounter = 0
        for port, desc, hwid in sorted(self.ports):
            # self.COMPortComboBox.addItem("[{}] {}: {}".format(i, port, desc))
            self.COMPortComboBox.addItem(desc)
            self.portNames.append("{}".format(port))
            self.comPortCounter += 1

    def connectToArduino(self):
        if not self.arduinoConnected and self.COMPortComboBox.currentText():
            self.GSArduino = Ground_Station_Arduino(self.portNames[self.COMPortComboBox.currentIndex()], 9600)
            self.errorMessageBox.setPlainText("connected to arduino!")
            self.arduinoConnected = True
        elif self.arduinoConnected:
            print("Arduino already connected")
            self.errorMessageBox.setPlainText("Arduino already connected")
        else:
            self.errorMessageBox.setPlainText("Unable to connect to Arduino")

        return

    def tiltUp(self):
        if self.arduinoConnected:
            self.GSArduino.adjustTiltUp(self.degreesPerClickBox.currentText())
            self.errorMessageBox.setPlainText("adjusting tilt up " + self.degreesPerClickBox.currentText() + " degrees")
        else:
            print("Unable to connect to ground station motors")
            self.errorMessageBox.setPlainText("Not connected to ground station motors")

        return

    def tiltDown(self):
        if self.arduinoConnected:
            self.GSArduino.adjustTiltDown(self.degreesPerClickBox.currentText())
            self.errorMessageBox.setPlainText("adjusting tilt down " + self.degreesPerClickBox.currentText() + " degrees")
        else:
            print("Unable to connect to ground station motors")
            self.errorMessageBox.setPlainText("Not connected to ground station motors")

        return

    def panCounterClockwise(self):
        if self.arduinoConnected:
            self.GSArduino.adjustPanNegative(self.degreesPerClickBox.currentText())
            self.errorMessageBox.setPlainText("adjusting pan " + self.degreesPerClickBox.currentText() + " degrees negative")
        else:
            print("Unable to connect to ground station motors")
            self.errorMessageBox.setPlainText("Not connected to ground station motors")

        return

    def panClockwise(self):
        if self.arduinoConnected:
            self.GSArduino.adjustPanPositive(self.degreesPerClickBox.currentText())
            self.errorMessageBox.setPlainText("adjusting pan " + self.degreesPerClickBox.currentText() + " degrees positive")
        else:
            print("Unable to connect to ground station motors")
            self.errorMessageBox.setPlainText("Not connected to ground station motors")

        return

    def getGSLocation(self):
        if self.arduinoConnected:
            check = self.GSArduino.warm_start()
            if not check:  # if the coords cannot be retrieved, return
                print("Failed to get GPS coords, please try again")
                self.errorMessageBox.setPlainText("Failed to get GPS coordinates, please try again")
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
            self.errorMessageBox.setPlainText("Arduino not connected")

        return

    def setGSLocation(self):
        try:
            if self.arduinoConnected:
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
                self.GSLocationSet = True
            else:
                self.errorMessageBox.setPlainText("Please connect arduino")
                self.GSLocationSet = False
        except ValueError:
            print("numbers only for GPS location")
            self.errorMessageBox.setPlainText("invalid GPS location entered. Only enter numbers")

    def getStartingPos(self):
        if self.GSLocationSet:
            now = datetime.utcnow()
            az, elev = sunpos(now, self.GSLat, self.GSLong, self.GSAlt)[:2]  # discard RA, dec, H

            self.startingAzimuth = az
            self.startingElevation = elev

            self.startingAzimuthBox.setPlainText(str(az))
            self.startingElevationBox.setPlainText(str(elev))

        else:
            self.errorMessageBox.setPlainText("Please set ground station location "
                                              "and point at the sun using solar sight")

        return

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
                self.errorMessageBox.setPlainText("Successfully calibrated!")
            except ValueError:
                print("numbers only for initial azimuth and elevation")
                self.errorMessageBox.setPlainText("Invalid input for initial azimuth and elevation")
        else:
            print("not connected to arduino")
            self.errorMessageBox.setPlainText("Not connected to arduino")

        return

    def setPredictTrack(self):
        self.predictingTrack = True
        self.checkIfReady()
        return

    def checkIfReady(self):
        if self.calibrated:
            print("Calibrated!")
        else:
            print("starting position not set")
            self.errorMessageBox.setPlainText("Please set staring azimuth and elevation")

        if self.GSLocationSet:
            print("Ground station location set!")
        else:
            print("Ground Station Location not assigned")
            self.errorMessageBox.setPlainText("Ground Station location not assigned")

        if self.IMEIAssigned:
            print("IMEI assigned")
        else:
            print("IMEI not assigned")
            self.errorMessageBox.setPlainText("Please select a balloon")

        if self.arduinoConnected:
            print("Arduino connected!")
        else:
            print("Please connect to the Arduino")
            self.errorMessageBox.setPlainText("Please connect to the Arduino")

        print("\n")

        if self.arduinoConnected and self.IMEIAssigned and self.calibrated and self.GSLocationSet:
            if self.predictingTrack and not DEBUG:
                self.errorMessageBox.setPlainText("Starting tracking with predictions!")
                self.predictTrack()
            elif self.predictingTrack and DEBUG:
                self.errorMessageBox.setPlainText("tracking with predictions debug mode")
                self.debugPredictTrack()
            else:
                self.errorMessageBox.setPlainText("starting tracking!")
                print("starting tracking!")
                self.track()
                return True
        else:
            print("not ready to track yet")
            return False

    def track(self):
        self.tracking = True
        self.errorMessageBox.setPlainText("Tracking!")
        self.trackThread = QThread()
        self.worker = Worker()

        self.worker.moveToThread(self.trackThread)

        self.trackThread.started.connect(self.worker.track)

        self.worker.finished.connect(self.trackThread.quit)  # pycharm has bug, this is correct
        self.worker.finished.connect(self.worker.deleteLater)  # https://youtrack.jetbrains.com/issue/PY-24183?_ga=2.240219907.1479555738.1625151876-2014881275.1622661488
        self.trackThread.finished.connect(self.trackThread.deleteLater)

        self.startButton.setEnabled(False)
        self.predictionStartButton.setEnabled(False)
        self.calibrateButton.setEnabled(False)

        self.trackThread.start()

    def predictTrack(self):
        self.tracking = True
        self.errorMessageBox.setPlainText("Tracking with predictions!")
        self.trackThread = QThread()
        self.worker = Worker()

        self.worker.moveToThread(self.trackThread)

        self.trackThread.started.connect(self.worker.predictTrack)

        self.worker.finished.connect(self.trackThread.quit)  # pycharm has bug, this is correct
        self.worker.finished.connect(self.worker.deleteLater)  # https://youtrack.jetbrains.com/issue/PY-24183?_ga=2.240219907.1479555738.1625151876-2014881275.1622661488
        self.trackThread.finished.connect(self.trackThread.deleteLater)

        self.startButton.setEnabled(False)
        self.predictionStartButton.setEnabled(False)
        self.calibrateButton.setEnabled(False)

        self.trackThread.start()

    def debugPredictTrack(self):
        self.tracking = True
        self.errorMessageBox.setPlainText("Tracking with predictions (debug)!")
        self.trackThread = QThread()
        self.worker = Worker()

        self.worker.moveToThread(self.trackThread)

        self.trackThread.started.connect(self.worker.debugPredictTrack)

        self.worker.finished.connect(self.trackThread.quit)  # pycharm has bug, this is correct
        self.worker.finished.connect(
            self.worker.deleteLater)  # https://youtrack.jetbrains.com/issue/PY-24183?_ga=2.240219907.1479555738.1625151876-2014881275.1622661488
        self.trackThread.finished.connect(self.trackThread.deleteLater)

        self.startButton.setEnabled(False)
        self.predictionStartButton.setEnabled(False)
        self.calibrateButton.setEnabled(False)

        self.trackThread.start()

    def stopTracking(self):
        # make the last command sent to the motors stop
        # want it to stop as soon as you hit stop tracking
        self.tracking = False
        self.predictingTrack = False
        self.startButton.setEnabled(True)
        self.predictionStartButton.setEnabled(True)
        self.calibrateButton.setEnabled(True)
        self.errorMessageBox.setPlainText("tracking stopped")
        return

    def displayCalculations(self, distance, azimuth, elevation):
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
                if not Balloon_Coor:
                    pass
                else:
                    # note that trackMath takes arguments as long, lat, altitude
                    Tracking_Calc = trackMath(MainWindow.GSLong, MainWindow.GSLat, MainWindow.GSAlt, Balloon_Coor[1],
                                              Balloon_Coor[0], Balloon_Coor[2])

                    distance = Tracking_Calc.distance
                    newElevation = Tracking_Calc.elevation()
                    newAzimuth = Tracking_Calc.azimuth()

                    print(str(self.i) + " Distance " + str(distance) + " Azimuth: " + str(newAzimuth) + ", Elevation: " + str(newElevation))

                    self.calcSignal.connect(MainWindow.displayCalculations)  # this seems to happen a lot for some reason
                    self.calcSignal.emit(distance, newAzimuth, newElevation)

                    MainWindow.GSArduino.move_position(newAzimuth, newElevation)

                    self.i += 1

        print("All done!")
        self.finished.emit()  # same pycharm bug as above
        return

    # check for new location from server
    # if the new location is still the same, go to prediction
    # if there is new location, go to that latest location

    # find the difference between the latest lat/long location and the one before the last one and time
    # using last vertical velocity/altitude, find difference between altitudes/new altitude after ~1 second
    # find the amount that the position is changing each ~second
    # input the new predicted lat/long/alt into math equations to get new azimuth/elevation
    # go to the predicted elevation/azimuth

    def predictTrack(self):
        print("In predictTrack")
        if DEBUG:
            self.debugPredictTrack()
            return

        timer = time.time()
        newestLocation = MainWindow.Balloon.get_coor_alt()
        oldLocation = MainWindow.Balloon.get_coor_alt()
        i = 1

        calculations = open("predictedOutput.csv", "w")
        csvWriter = csv.writer(calculations)
        calcFields = ["Distance", "Azimuth", "Elevation", "r/p"]
        csvWriter.writerow(calcFields)

        azimuthList = []
        elevationList = []

        while MainWindow.predictingTrack:
            if (time.time() - timer) > 1:
                timer = time.time()
                currData = MainWindow.Balloon.get_coor_alt()

                if newestLocation == currData:
                    # need to predict!
                    print("predicted output")
                    timeDelta = MainWindow.Balloon.getTimeDiff()
                    print("The time delta is: " + str(timeDelta))
                    latStep = (newestLocation[0] - oldLocation[0]) / timeDelta
                    longStep = (newestLocation[1] - oldLocation[1]) / timeDelta
                    altStep = (newestLocation[2] - oldLocation[2]) / timeDelta

                    Tracking_Calc = trackMath(MainWindow.GSLong, MainWindow.GSLat, MainWindow.GSAlt,
                                              newestLocation[1] + (i * longStep), newestLocation[0] + (i * latStep), newestLocation[2] + (i * altStep))

                    distance = Tracking_Calc.distance
                    newElevation = Tracking_Calc.elevation()
                    newAzimuth = Tracking_Calc.azimuth()

                    elevationList.append(newElevation)
                    azimuthList.append(newAzimuth)

                    # keep average of azimuth/elevations
                    # if new calculation is outlier, throw it out, don't go to new spot
                    # reset average between pings
                    # alternatively, implement some type of filter (savitzky golay, kalman, etc)

                    if newElevation > np.mean(elevationList) + (2 * np.std(elevationList)) or newElevation < np.mean(elevationList) - (2 * np.std(elevationList)) \
                            or newAzimuth > np.mean(azimuthList) + (2 * np.std(azimuthList)) or newAzimuth < np.mean(azimuthList) - (2 * np.std(azimuthList)):
                        print("outlier detected! ")
                        pass
                    else:
                        print("distance: " + str(distance))
                        print("elevation: " + str(newElevation))
                        print("azimuth: " + str(newAzimuth) + "\n")

                        self.calcSignal.connect(MainWindow.displayCalculations)
                        self.calcSignal.emit(distance, newAzimuth, newElevation)

                        row = [distance, newAzimuth, newElevation, "p"]
                        csvWriter.writerow(row)

                        # MainWindow.GSArduino.move_position(newAzimuth, newElevation)

                    i += 1

                else:
                    # go to the new actual spot
                    oldLocation = newestLocation
                    newestLocation = currData

                    # note that trackMath takes arguments as long, lat, altitude
                    Tracking_Calc = trackMath(MainWindow.GSLong, MainWindow.GSLat, MainWindow.GSAlt, currData[1],
                                              currData[0], currData[2])

                    distance = Tracking_Calc.distance
                    newElevation = Tracking_Calc.elevation()
                    newAzimuth = Tracking_Calc.azimuth()

                    self.calcSignal.connect(MainWindow.displayCalculations)  # this seems to happen a lot for some reason
                    self.calcSignal.emit(distance, newAzimuth, newElevation)

                    print("Got new real ping!")
                    print("distance: " + str(distance))
                    print("elevation: " + str(newElevation))
                    print("azimuth: " + str(newAzimuth) + "\n")

                    # MainWindow.GSArduino.move_position(newAzimuth, newElevation)

                    row = [distance, newAzimuth, newElevation, "r"]
                    csvWriter.writerow(row)

                    i = 1
                    azimuthList = []
                    elevationList = []

        print("All done tracking with predictions! :)")
        calculations.close()
        return

    def debugPredictTrack(self):
        print("In debug predict track")
        timeDiffs = fakeIridium.getTimeDiffs()
        print(timeDiffs)

        lats = fakeIridium.getLats()
        print(lats)
        longs = fakeIridium.getLongs()
        print(longs)
        alts = fakeIridium.getAlts()
        print(alts)

        timer = time.time()
        timeDiffCounter = time.time()
        index = 1

        calculations = open("predictedOutput.csv", "w")
        csvWriter = csv.writer(calculations)
        calcFields = ["Distance", "Azimuth", "Elevation", "r/p"]
        csvWriter.writerow(calcFields)

        azimuthList = []
        elevationList = []

        newestLocation = oldLocation = [lats[1], longs[1], alts[1]]

        i = 1

        while MainWindow.predictingTrack:
            if (time.time() - timer) > 1:
                timer = time.time()
                # currData = [lats[index], longs[index], alts[index]]

                if (time.time() - timeDiffCounter) < timeDiffs[index]:
                    # need to predict!
                    print("predicted output")
                    timeDelta = timeDiffs[index]
                    print("The time delta is: " + str(timeDelta))

                    latStep = (float(newestLocation[0]) - float(oldLocation[0])) / float(timeDelta)
                    print("lat step: " + str(latStep))
                    longStep = (float(newestLocation[1]) - float(oldLocation[1])) / float(timeDelta)
                    print("long step: " + str(longStep))
                    altStep = (float(newestLocation[2]) - float(oldLocation[2])) / float(timeDelta)
                    print("alt step: " + str(altStep))

                    Tracking_Calc = trackMath(MainWindow.GSLong, MainWindow.GSLat, MainWindow.GSAlt,
                                              float(newestLocation[1]) + (i * longStep), float(newestLocation[0]) + (i * latStep),
                                              float(newestLocation[2]) + (i * altStep))

                    distance = Tracking_Calc.distance
                    print("distance :" + str(distance))
                    newElevation = Tracking_Calc.elevation()
                    print("elevation: " + str(newElevation))
                    newAzimuth = Tracking_Calc.azimuth()
                    print("azimuth: " + str(newAzimuth))

                    elevationList.append(newElevation)
                    azimuthList.append(newAzimuth)

                    # keep average of azimuth/elevations
                    # if new calculation is outlier, throw it out, don't go to new spot
                    # reset average between pings
                    # alternatively, implement some type of filter (savitzky golay, kalman, etc)

                    # if newElevation > np.mean(elevationList) + (2 * np.std(elevationList)) or newElevation < np.mean(
                    #         elevationList) - (2 * np.std(elevationList)) \
                    #         or newAzimuth > np.mean(azimuthList) + (2 * np.std(azimuthList)) or newAzimuth < np.mean(
                    #     azimuthList) - (2 * np.std(azimuthList)):
                    #     print("outlier detected! ")
                    #     pass
                    # else:

                    self.calcSignal.connect(MainWindow.displayCalculations)
                    self.calcSignal.emit(distance, newAzimuth, newElevation)

                    row = [distance, newAzimuth, newElevation, "p"]
                    csvWriter.writerow(row)

                    # MainWindow.GSArduino.move_position(newAzimuth, newElevation)

                    i += 1

                    print("\n")

                else:
                    # go to the new actual spot
                    index += 1
                    oldLocation = newestLocation
                    newestLocation = [lats[index], longs[index], alts[index]]

                    # note that trackMath takes arguments as long, lat, altitude
                    Tracking_Calc = trackMath(MainWindow.GSLong, MainWindow.GSLat, MainWindow.GSAlt, float(newestLocation[1]),
                                              float(newestLocation[0]), float(newestLocation[2]))

                    distance = Tracking_Calc.distance
                    newElevation = Tracking_Calc.elevation()
                    newAzimuth = Tracking_Calc.azimuth()

                    self.calcSignal.connect(
                        MainWindow.displayCalculations)  # this seems to happen a lot for some reason
                    self.calcSignal.emit(distance, newAzimuth, newElevation)

                    print("Got new real ping!")
                    print("distance: " + str(distance))
                    print("elevation: " + str(newElevation))
                    print("azimuth: " + str(newAzimuth) + "\n")

                    # MainWindow.GSArduino.move_position(newAzimuth, newElevation)

                    row = [distance, newAzimuth, newElevation, "r"]
                    csvWriter.writerow(row)

                    i = 1
                    azimuthList = []
                    elevationList = []

                    index += 1

                    timeDiffCounter = time.time()

        print("All done tracking with predictions! :)")
        calculations.close()


        # implement normal predict track function
        # replace timediff check from website to using the list
        # if the current time diff is less than the next entry in list, make prediction
        # otherwise go to the next location in the csv file

        return


if __name__ == "__main__":
    if DEBUG:
        import fakeIridium

    app = QtWidgets.QApplication(sys.argv)
    MainWindow = Window()
    # MainWindow.show()
    MainWindow.showMaximized()

    sys.exit(app.exec_())
