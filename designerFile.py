# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file '.\correctGridLayout.ui'
#
# Created by: PyQt5 UI code generator 5.15.4
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(785, 700)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.gridLayout_5 = QtWidgets.QGridLayout(self.centralwidget)
        self.gridLayout_5.setObjectName("gridLayout_5")
        self.gridLayout_3 = QtWidgets.QGridLayout()
        self.gridLayout_3.setObjectName("gridLayout_3")
        self.tiltUpButton = QtWidgets.QPushButton(self.centralwidget)
        self.tiltUpButton.setObjectName("tiltUpButton")
        self.gridLayout_3.addWidget(self.tiltUpButton, 1, 3, 1, 1)
        self.label_13 = QtWidgets.QLabel(self.centralwidget)
        self.label_13.setObjectName("label_13")
        self.gridLayout_3.addWidget(self.label_13, 0, 0, 1, 1)
        self.panCounterClockwiseButton = QtWidgets.QPushButton(self.centralwidget)
        self.panCounterClockwiseButton.setObjectName("panCounterClockwiseButton")
        self.gridLayout_3.addWidget(self.panCounterClockwiseButton, 2, 2, 1, 1)
        self.tiltDownButton = QtWidgets.QPushButton(self.centralwidget)
        self.tiltDownButton.setObjectName("tiltDownButton")
        self.gridLayout_3.addWidget(self.tiltDownButton, 3, 3, 1, 1)
        self.panClockwiseButton = QtWidgets.QPushButton(self.centralwidget)
        self.panClockwiseButton.setObjectName("panClockwiseButton")
        self.gridLayout_3.addWidget(self.panClockwiseButton, 2, 4, 1, 1)
        self.degreesPerClickBox = QtWidgets.QComboBox(self.centralwidget)
        self.degreesPerClickBox.setSizeAdjustPolicy(QtWidgets.QComboBox.AdjustToContents)
        self.degreesPerClickBox.setObjectName("degreesPerClickBox")
        self.degreesPerClickBox.addItem("")
        self.degreesPerClickBox.addItem("")
        self.degreesPerClickBox.addItem("")
        self.degreesPerClickBox.addItem("")
        self.gridLayout_3.addWidget(self.degreesPerClickBox, 0, 1, 1, 1)
        self.gridLayout_5.addLayout(self.gridLayout_3, 1, 0, 1, 1)
        self.gridLayout_2 = QtWidgets.QGridLayout()
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.GSAltBox = QtWidgets.QPlainTextEdit(self.centralwidget)
        self.GSAltBox.setObjectName("GSAltBox")
        self.gridLayout_2.addWidget(self.GSAltBox, 2, 1, 1, 1)
        spacerItem = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.gridLayout_2.addItem(spacerItem, 12, 0, 1, 1)
        self.label_2 = QtWidgets.QLabel(self.centralwidget)
        self.label_2.setObjectName("label_2")
        self.gridLayout_2.addWidget(self.label_2, 1, 0, 1, 1)
        self.GPSRequestButton = QtWidgets.QPushButton(self.centralwidget)
        self.GPSRequestButton.setObjectName("GPSRequestButton")
        self.gridLayout_2.addWidget(self.GPSRequestButton, 0, 2, 1, 1)
        self.label = QtWidgets.QLabel(self.centralwidget)
        self.label.setObjectName("label")
        self.gridLayout_2.addWidget(self.label, 0, 0, 1, 1)
        self.label_3 = QtWidgets.QLabel(self.centralwidget)
        self.label_3.setObjectName("label_3")
        self.gridLayout_2.addWidget(self.label_3, 2, 0, 1, 1)
        self.label_5 = QtWidgets.QLabel(self.centralwidget)
        self.label_5.setObjectName("label_5")
        self.gridLayout_2.addWidget(self.label_5, 5, 0, 1, 1)
        self.label_8 = QtWidgets.QLabel(self.centralwidget)
        self.label_8.setObjectName("label_8")
        self.gridLayout_2.addWidget(self.label_8, 9, 0, 1, 1)
        self.label_9 = QtWidgets.QLabel(self.centralwidget)
        self.label_9.setObjectName("label_9")
        self.gridLayout_2.addWidget(self.label_9, 10, 0, 1, 1)
        self.label_6 = QtWidgets.QLabel(self.centralwidget)
        self.label_6.setObjectName("label_6")
        self.gridLayout_2.addWidget(self.label_6, 6, 0, 1, 1)
        self.label_10 = QtWidgets.QLabel(self.centralwidget)
        self.label_10.setObjectName("label_10")
        self.gridLayout_2.addWidget(self.label_10, 11, 0, 1, 1)
        self.label_12 = QtWidgets.QLabel(self.centralwidget)
        self.label_12.setObjectName("label_12")
        self.gridLayout_2.addWidget(self.label_12, 14, 0, 1, 1)
        spacerItem1 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.gridLayout_2.addItem(spacerItem1, 7, 0, 1, 1)
        self.GSLatBox = QtWidgets.QPlainTextEdit(self.centralwidget)
        self.GSLatBox.setObjectName("GSLatBox")
        self.gridLayout_2.addWidget(self.GSLatBox, 0, 1, 1, 1)
        self.GSLongBox = QtWidgets.QPlainTextEdit(self.centralwidget)
        self.GSLongBox.setObjectName("GSLongBox")
        self.gridLayout_2.addWidget(self.GSLongBox, 1, 1, 1, 1)
        self.label_15 = QtWidgets.QLabel(self.centralwidget)
        self.label_15.setText("")
        self.label_15.setObjectName("label_15")
        self.gridLayout_2.addWidget(self.label_15, 13, 1, 1, 1)
        self.startingAzimuthBox = QtWidgets.QPlainTextEdit(self.centralwidget)
        self.startingAzimuthBox.setObjectName("startingAzimuthBox")
        self.gridLayout_2.addWidget(self.startingAzimuthBox, 5, 1, 1, 1)
        self.calibrateButton = QtWidgets.QPushButton(self.centralwidget)
        self.calibrateButton.setObjectName("calibrateButton")
        self.gridLayout_2.addWidget(self.calibrateButton, 6, 2, 1, 1)
        self.startingElevationBox = QtWidgets.QPlainTextEdit(self.centralwidget)
        self.startingElevationBox.setObjectName("startingElevationBox")
        self.gridLayout_2.addWidget(self.startingElevationBox, 6, 1, 1, 1)
        self.distanceDisplay = QtWidgets.QPlainTextEdit(self.centralwidget)
        self.distanceDisplay.setObjectName("distanceDisplay")
        self.gridLayout_2.addWidget(self.distanceDisplay, 9, 1, 1, 1)
        self.azimuthDisplay = QtWidgets.QPlainTextEdit(self.centralwidget)
        self.azimuthDisplay.setObjectName("azimuthDisplay")
        self.gridLayout_2.addWidget(self.azimuthDisplay, 10, 1, 1, 1)
        self.elevationDisplay = QtWidgets.QPlainTextEdit(self.centralwidget)
        self.elevationDisplay.setObjectName("elevationDisplay")
        self.gridLayout_2.addWidget(self.elevationDisplay, 11, 1, 1, 1)
        self.errorMessageBox = QtWidgets.QPlainTextEdit(self.centralwidget)
        self.errorMessageBox.setObjectName("errorMessageBox")
        self.gridLayout_2.addWidget(self.errorMessageBox, 14, 1, 1, 1)
        self.calculateStartingPosButton = QtWidgets.QPushButton(self.centralwidget)
        self.calculateStartingPosButton.setObjectName("calculateStartingPosButton")
        self.gridLayout_2.addWidget(self.calculateStartingPosButton, 5, 2, 1, 1)
        self.label_7 = QtWidgets.QLabel(self.centralwidget)
        self.label_7.setObjectName("label_7")
        self.gridLayout_2.addWidget(self.label_7, 4, 2, 1, 1)
        spacerItem2 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.gridLayout_2.addItem(spacerItem2, 3, 1, 1, 1)
        self.confirmGSLocationButton = QtWidgets.QPushButton(self.centralwidget)
        self.confirmGSLocationButton.setObjectName("confirmGSLocationButton")
        self.gridLayout_2.addWidget(self.confirmGSLocationButton, 2, 2, 1, 1)
        spacerItem3 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.gridLayout_2.addItem(spacerItem3, 12, 1, 1, 1)
        self.label_16 = QtWidgets.QLabel(self.centralwidget)
        self.label_16.setText("")
        self.label_16.setObjectName("label_16")
        self.gridLayout_2.addWidget(self.label_16, 8, 0, 1, 1)
        self.gridLayout_5.addLayout(self.gridLayout_2, 0, 2, 4, 1)
        self.gridLayout_4 = QtWidgets.QGridLayout()
        self.gridLayout_4.setObjectName("gridLayout_4")
        self.stopButton = QtWidgets.QPushButton(self.centralwidget)
        self.stopButton.setObjectName("stopButton")
        self.gridLayout_4.addWidget(self.stopButton, 1, 0, 1, 2)
        self.predictionStartButton = QtWidgets.QPushButton(self.centralwidget)
        self.predictionStartButton.setObjectName("predictionStartButton")
        self.gridLayout_4.addWidget(self.predictionStartButton, 0, 1, 1, 1)
        self.startButton = QtWidgets.QPushButton(self.centralwidget)
        self.startButton.setObjectName("startButton")
        self.gridLayout_4.addWidget(self.startButton, 0, 0, 1, 1)
        self.gridLayout_5.addLayout(self.gridLayout_4, 3, 0, 1, 1)
        spacerItem4 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.gridLayout_5.addItem(spacerItem4, 1, 1, 1, 1)
        self.gridLayout = QtWidgets.QGridLayout()
        self.gridLayout.setObjectName("gridLayout")
        self.label_4 = QtWidgets.QLabel(self.centralwidget)
        self.label_4.setObjectName("label_4")
        self.gridLayout.addWidget(self.label_4, 0, 0, 1, 1)
        self.IMEIComboBox = QtWidgets.QComboBox(self.centralwidget)
        self.IMEIComboBox.setObjectName("IMEIComboBox")
        self.gridLayout.addWidget(self.IMEIComboBox, 1, 0, 1, 1)
        self.connectToArduinoButton = QtWidgets.QPushButton(self.centralwidget)
        self.connectToArduinoButton.setObjectName("connectToArduinoButton")
        self.gridLayout.addWidget(self.connectToArduinoButton, 3, 1, 1, 1)
        self.label_11 = QtWidgets.QLabel(self.centralwidget)
        self.label_11.setObjectName("label_11")
        self.gridLayout.addWidget(self.label_11, 2, 0, 1, 1)
        self.COMPortComboBox = QtWidgets.QComboBox(self.centralwidget)
        self.COMPortComboBox.setObjectName("COMPortComboBox")
        self.gridLayout.addWidget(self.COMPortComboBox, 3, 0, 1, 1)
        self.confirmIMEIButton = QtWidgets.QPushButton(self.centralwidget)
        self.confirmIMEIButton.setObjectName("confirmIMEIButton")
        self.gridLayout.addWidget(self.confirmIMEIButton, 1, 1, 1, 1)
        spacerItem5 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.gridLayout.addItem(spacerItem5, 4, 0, 1, 1)
        self.gridLayout_5.addLayout(self.gridLayout, 0, 0, 1, 1)
        spacerItem6 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.gridLayout_5.addItem(spacerItem6, 2, 0, 1, 1)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 785, 20))
        self.menubar.setObjectName("menubar")
        self.menuBRAD_Station_Tracking_Software = QtWidgets.QMenu(self.menubar)
        self.menuBRAD_Station_Tracking_Software.setObjectName("menuBRAD_Station_Tracking_Software")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)
        self.menubar.addAction(self.menuBRAD_Station_Tracking_Software.menuAction())

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.tiltUpButton.setText(_translate("MainWindow", "             Tilt Up             "))
        self.label_13.setText(_translate("MainWindow", "Degrees Per Click:"))
        self.panCounterClockwiseButton.setText(_translate("MainWindow", "Pan Counter-Clockwise"))
        self.tiltDownButton.setText(_translate("MainWindow", "Tilt Down"))
        self.panClockwiseButton.setText(_translate("MainWindow", "       Pan Clockwise       "))
        self.degreesPerClickBox.setItemText(0, _translate("MainWindow", ".5"))
        self.degreesPerClickBox.setItemText(1, _translate("MainWindow", "1"))
        self.degreesPerClickBox.setItemText(2, _translate("MainWindow", "5"))
        self.degreesPerClickBox.setItemText(3, _translate("MainWindow", "10"))
        self.label_2.setText(_translate("MainWindow", "Ground Station Longitude: "))
        self.GPSRequestButton.setText(_translate("MainWindow", "Request Ground Station Location"))
        self.label.setText(_translate("MainWindow", "Ground Station Latitude: "))
        self.label_3.setText(_translate("MainWindow", "Ground Station Altitude (m): "))
        self.label_5.setText(_translate("MainWindow", "Starting Azimuth: "))
        self.label_8.setText(_translate("MainWindow", "Line of Sight Distance (m): "))
        self.label_9.setText(_translate("MainWindow", "Azimuth: "))
        self.label_6.setText(_translate("MainWindow", "Starting Elevation:"))
        self.label_10.setText(_translate("MainWindow", "Elevation: "))
        self.label_12.setText(_translate("MainWindow", "Status Box:"))
        self.calibrateButton.setText(_translate("MainWindow", "Set Starting Position"))
        self.calculateStartingPosButton.setText(_translate("MainWindow", "Calculate Starting Position"))
        self.label_7.setText(_translate("MainWindow", "When pointed at the Sun, \n"
" click the button below"))
        self.confirmGSLocationButton.setText(_translate("MainWindow", "Set Ground Station Location"))
        self.stopButton.setText(_translate("MainWindow", "Stop Tracking"))
        self.predictionStartButton.setText(_translate("MainWindow", "Start Tracking \n"
" with Prediction"))
        self.startButton.setText(_translate("MainWindow", "Start Basic \n"
" Tracking"))
        self.label_4.setText(_translate("MainWindow", "Select or Enter IMEI Below"))
        self.connectToArduinoButton.setText(_translate("MainWindow", "Connect to Arduino"))
        self.label_11.setText(_translate("MainWindow", "Select Arduino From the List Below"))
        self.confirmIMEIButton.setText(_translate("MainWindow", "Confirm IMEI"))
        self.menuBRAD_Station_Tracking_Software.setTitle(_translate("MainWindow", "BRAD Station Tracking Software"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
