from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QTimer, QThread, QObject, pyqtSignal
from designerFile import Ui_MainWindow
import sys
from Balloon_Coordinates import Balloon_Coordinates
from Ground_Station_Coordinates import Ground_Station_Coordinates
from satelliteTrackingMath import trackMath
from Ground_Station_Motors import Ground_Station_Motors
import serial.tools.list_ports
import time


# todo: set up git repo/other backup
# todo: input validation
# todo: make selecting IMEI/COM port easier
# todo: clean up code (Ground_Station_Motors)
# todo: display balloon info and calculation
# todo: display error messages in GUI
# todo: make window scale/look good (bigger text)
# todo: implement map of balloon location


class Window(QtWidgets.QMainWindow, Ui_MainWindow):
    def __init__(self):
        super(Window, self).__init__()
        self.setupUi(self)

        self.IMEIList = Balloon_Coordinates.list_IMEI()

        self.ports = serial.tools.list_ports.comports()
        self.portNames = []

        self.timer = QTimer()

        self.arduinoConnected = False
        self.IMEIAssigned = False

        self.tracking = False

        self.GSMotors = None  # classes will be instantiated later
        self.Balloon = None

        self.trackThread = None
        self.worker = None

        self.GSLat = 0
        self.GSLong = 0
        self.GSAlt = 0

        for i in range(len(self.IMEIList)):
            self.IMEIComboBox.addItem(self.IMEIList[i])

        i = 1
        for port, desc, hwid in sorted(self.ports):
            self.COMPortComboBox.addItem("[{}] {}: {}".format(i, port, desc))
            self.portNames.append("{}".format(port))
            i += 1

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
        else:
            print("select a balloon ")
            self.IMEIAssigned = False
        return

    """
    def assignCOMPort(self):
        user_port = self.COMPortComboBox.currentIndex()
        COMPortAssigned = True
        print(user_port)
        return True
    """

    def connectToArduino(self):
        if not self.arduinoConnected:
            self.GSMotors = Ground_Station_Motors(self.portNames[self.COMPortComboBox.currentIndex() - 1], 9600)
        self.arduinoConnected = True
        return

    def tiltUp(self):
        if self.arduinoConnected:
            self.GSMotors.adjustTiltUp()
            print("adjusting tilt up 1 degree")
        else:
            print("Unable to connect to ground station motors")

        return

    def tiltDown(self):
        if self.arduinoConnected:
            self.GSMotors.adjustTiltDown()
            print("adjusting tilt down 1 degree")
        else:
            print("Unable to connect to ground station motors")

        return

    def panLeft(self):
        if self.arduinoConnected:
            self.GSMotors.adjustPanNegative()
            print("adjusting pan 1 degree negative")
        else:
            print("Unable to connect to ground station motors")

        return

    def panRight(self):
        if self.arduinoConnected:
            self.GSMotors.adjustPanPositive()
            print("adjusting pan 1 degree positive")
        else:
            print("Unable to connect to ground station motors")

        return

    def getGSLocation(self):
        if self.arduinoConnected:
            self.GSMotors.warm_start()  # todo handle if arduino doesn't get gps location
            time.sleep(1)

            GSCoords = self.GSMotors.req_GPS()
            self.GSMotors.print_GPS()
            self.GSLat = GSCoords[0]
            self.GSLong = GSCoords[1]
            self.GSAlt = GSCoords[2]

            self.GSLatBox.setPlainText(str(self.GSLat))
            self.GSLongBox.setPlainText(str(self.GSLong))
            self.GSAltBox.setPlainText(str(self.GSAlt))
        else:
            print("arduino not connected")

        return

    def setGSLocation(self):
        latStr = self.GSLatBox.toPlainText()  # todo validate the input
        # latStr = latStr.strip()
        self.GSLat = float(latStr)
        print(self.GSLat)

        longStr = self.GSLongBox.toPlainText()
        self.GSLong = float(longStr)
        print(self.GSLong)

        altStr = self.GSAltBox.toPlainText()
        self.GSAlt = float(altStr)
        print(self.GSAlt)

    def calibrate(self):
        if self.arduinoConnected:
            startingAzimuthStr = self.startingAzimuthBox.toPlainText()  # todo validate input
            startingAzimuth = float(startingAzimuthStr)
            print(startingAzimuth)

            startingElevationStr = self.startingElevationBox.toPlainText()
            startingElevation = float(startingElevationStr)
            print(startingElevation)

            self.GSMotors.calibrate(startingAzimuth, startingElevation)
        else:
            print("not connected to arduino")

        return

    def checkIfReady(self):
        if self.arduinoConnected:
            print("Com port assigned")
        else:
            print("Com port not assigned")

        if self.IMEIAssigned:
            print("IMEI assigned")
        else:
            print("IMEI not assigned")

        print("\n")

        if self.arduinoConnected and self.IMEIAssigned != 0:
            self.track()
            return True
        else:
            return False

    def track(self):
        self.tracking = True
        self.trackThread = QThread()
        self.worker = Worker()

        self.worker.moveToThread(self.trackThread)

        # self.trackThread.started.connect(lambda: self.worker.track(self.GSMotors, self.Balloon))
        self.trackThread.started.connect(self.worker.track)
        # self.trackThread.started.connect(self.worker.quickTest)

        self.worker.finished.connect(self.trackThread.quit)  # pycharm has bug, this is correct
        self.worker.finished.connect(self.worker.deleteLater)  # https://youtrack.jetbrains.com/issue/PY-24183?_ga=2.240219907.1479555738.1625151876-2014881275.1622661488
        self.trackThread.finished.connect(self.trackThread.deleteLater)

        self.trackThread.start()

    def stopTracking(self):
        self.tracking = False
        return


class Worker(QObject):
    finished = pyqtSignal()

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

                print("Distance " + str(distance) + " Azimuth: " + str(newAzimuth) + ", Elevation: " + str(newElevation))

                MainWindow.GSMotors.move_position(newAzimuth, newElevation)

        print("All done!")
        self.finished.emit()
        return


if __name__ == "__main__":

    app = QtWidgets.QApplication(sys.argv)
    MainWindow = Window()
    MainWindow.show()

    sys.exit(app.exec_())
