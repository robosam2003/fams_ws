# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'Interface.ui'
##
## Created by: Qt User Interface Compiler version 6.6.1
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtWidgets import (QApplication, QComboBox, QFormLayout, QFrame,
    QLabel, QLineEdit, QListWidget, QListWidgetItem,
    QMainWindow, QPushButton, QSizePolicy, QStatusBar,
    QWidget)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        mainWindowSize=[752, 695]
        MainWindow.resize(mainWindowSize[0],mainWindowSize[1])

        xLabel = 20
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.stopButton = QPushButton(self.centralwidget)
        self.stopButton.setObjectName(u"stopButton")
        self.stopButton.setGeometry(QRect(165, 15, 500, 25))    # xLocation, yLocation, Width, Height (From top left)
        self.stopButton.setAutoRepeat(False)
        self.stopButton.setStyleSheet("background-color: rgb(255, 0, 0)")
        self.JobID = QLabel(self.centralwidget)
        self.JobID.setObjectName(u"JobID")
        self.JobID.setGeometry(QRect(xLabel, 84, 44, 17))
        self.job_id = QLineEdit(self.centralwidget)
        self.job_id.setObjectName(u"job_id")
        self.job_id.setGeometry(QRect(165, 84, 500, 25))
        self.PartID = QLabel(self.centralwidget)
        self.PartID.setObjectName(u"PartID")
        self.PartID.setGeometry(QRect(xLabel, 127, 47, 17))
        self.part_id = QLineEdit(self.centralwidget)
        self.part_id.setObjectName(u"part_id")
        self.part_id.setGeometry(QRect(165, 127, 500, 25))
        self.PartLocation = QLabel(self.centralwidget)
        self.PartLocation.setObjectName(u"PartLocation")
        self.PartLocation.setGeometry(QRect(xLabel, 164, 91, 17))
        self.partLocation = QLineEdit(self.centralwidget)
        self.partLocation.setObjectName(u"partLocation")
        self.partLocation.setGeometry(QRect(165, 164, 500, 25))
        self.SubprocessID = QLabel(self.centralwidget)
        self.SubprocessID.setObjectName(u"SubprocessID")
        self.SubprocessID.setGeometry(QRect(xLabel+10, 209, 96, 17))
        self.subID = QLineEdit(self.centralwidget)
        self.subID.setObjectName(u"subID")
        self.subID.setGeometry(QRect(165, 209, 480, 25))
        self.OperationType = QLabel(self.centralwidget)
        self.OperationType.setObjectName(u"OperationType")
        self.OperationType.setGeometry(QRect(100, 240, 106, 17))
        self.opType = QLineEdit(self.centralwidget)
        self.opType.setObjectName(u"opType")
        self.opType.setGeometry(QRect(265, 240, 380, 25))
        self.SubprocessStartTime = QLabel(self.centralwidget)
        self.SubprocessStartTime.setObjectName(u"SubprocessStartTime")
        self.SubprocessStartTime.setGeometry(QRect(100, 271, 150, 17))
        self.subStartTime = QLineEdit(self.centralwidget)
        self.subStartTime.setObjectName(u"subStartTime")
        self.subStartTime.setGeometry(QRect(265, 271, 380, 25))
        self.SubprocessEndTime = QLabel(self.centralwidget)
        self.SubprocessEndTime.setObjectName(u"SubprocessEndTime")
        self.SubprocessEndTime.setGeometry(QRect(100, 302, 145, 17))
        self.subEndTime = QLineEdit(self.centralwidget)
        self.subEndTime.setObjectName(u"subEndTime")
        self.subEndTime.setGeometry(QRect(265, 302, 380, 25))
        self.addToList = QPushButton(self.centralwidget)
        self.addToList.setObjectName(u"addToList")
        self.addToList.setGeometry(QRect(165, 333, 480, 25))
        self.addToList.setStyleSheet("background-color: rgb(70, 100, 200)")
        self.JobStartTime = QLabel(self.centralwidget)
        self.JobStartTime.setObjectName(u"JobStartTime")
        self.JobStartTime.setGeometry(QRect(xLabel, 374, 100, 17))
        self.startTime = QLineEdit(self.centralwidget)
        self.startTime.setObjectName(u"startTime")
        self.startTime.setGeometry(QRect(165, 374, 500, 25))
        self.JobEndTime = QLabel(self.centralwidget)
        self.JobEndTime.setObjectName(u"JobEndTime")
        self.JobEndTime.setGeometry(QRect(xLabel, 407, 92, 17))
        self.endTime = QLineEdit(self.centralwidget)
        self.endTime.setObjectName(u"endTime")
        self.endTime.setGeometry(QRect(165, 407, 500, 25))
        self.JobPriority = QLabel(self.centralwidget)
        self.JobPriority.setObjectName(u"JobPriority")
        self.JobPriority.setGeometry(QRect(xLabel, 440, 81, 17))
        self.priority = QComboBox(self.centralwidget)
        self.priority.addItem("")
        self.priority.addItem("")
        self.priority.addItem("")
        self.priority.setObjectName(u"priority")
        self.priority.setGeometry(QRect(165, 440, 38, 25))
        self.addJobButton = QPushButton(self.centralwidget)
        self.addJobButton.setObjectName(u"addJobButton")
        self.addJobButton.setGeometry(QRect(165, 473, 500, 25))
        self.addJobButton.setStyleSheet("background-color: rgb(60, 60, 80)")
        self.ActiveJobList = QLabel(self.centralwidget)
        self.ActiveJobList.setObjectName(u"ActiveJobList")
        self.ActiveJobList.setGeometry(QRect(10, 538, 99, 17))

        wJobWidget=235
        wSubWidget=200
        xSubWidget1=10+wJobWidget+xLabel
        xSubWidget2=xSubWidget1+wSubWidget+10
        self.jobListWidget = QListWidget(self.centralwidget)
        self.jobListWidget.setObjectName(u"JobListWidget")
        self.jobListWidget.setGeometry(QRect(10, 560, wJobWidget, 72))
        self.JobDeleteButton = QPushButton(self.centralwidget)
        self.JobDeleteButton.setObjectName(u"JobDeleteButton")
        self.JobDeleteButton.setGeometry(QRect(10, 639, wJobWidget, 25))
        self.SubprocessList = QLabel(self.centralwidget)
        self.SubprocessList.setObjectName(u"SubprocessList")
        self.SubprocessList.setGeometry(QRect(xSubWidget1, 538, 110, 17))
        
        self.subprocessListWidget_1 = QListWidget(self.centralwidget)
        self.subprocessListWidget_1.setObjectName(u"listWidget_1")
        self.subprocessListWidget_1.setGeometry(QRect(xSubWidget1, 560, wSubWidget, 72))
        self.subprocessListWidget_2 = QListWidget(self.centralwidget)
        self.subprocessListWidget_2.setObjectName(u"listWidget_2")
        self.subprocessListWidget_2.setGeometry(QRect(xSubWidget2, 560, wSubWidget, 72))

        self.frame = QFrame(self.centralwidget)     # Subprocess Frame
        self.frame.setObjectName(u"frame")
        self.frame.setGeometry(QRect(xLabel, 200, 645, 168))
        self.frame.setAutoFillBackground(False)
        self.frame.setFrameShape(QFrame.Panel)
        self.frame.setFrameShadow(QFrame.Raised)
        self.frame.setLineWidth(3)
        self.frame.setMidLineWidth(2)
        self.frame.setStyleSheet("background-color: rgb(125, 155, 200)")
        self.widget = QWidget(self.centralwidget)
        self.widget.setObjectName(u"widget")
        self.widget.setGeometry(QRect(0, 0, mainWindowSize[0], mainWindowSize[1]))
        self.widget.setStyleSheet("background-color: rgb(155, 155, 155)") # background colour of widget that covers whole background
        self.frame1 = QFrame(self.centralwidget)    # Job Frame
        self.frame1.setObjectName(u"frame")
        self.frame1.setGeometry(QRect(10, 74, 665, 442))
        self.frame1.setFrameShape(QFrame.Panel)
        self.frame1.setFrameShadow(QFrame.Raised)
        self.frame1.setLineWidth(3)
        self.frame1.setMidLineWidth(2)
        self.frame1.setStyleSheet("background-color: rgb(100, 100, 120)")
        self.formLayout = QFormLayout(self.widget)
        self.formLayout.setObjectName(u"formLayout")
        self.formLayout.setContentsMargins(0, 0, 0, 0)
        MainWindow.setCentralWidget(self.centralwidget)
        self.widget.raise_()    # order of raise_ 's determines what overlaps what
        self.frame1.raise_()
        self.frame.raise_()
        self.stopButton.raise_()
        self.JobID.raise_()
        self.job_id.raise_()
        self.PartID.raise_()
        self.part_id.raise_()
        self.PartLocation.raise_()
        self.partLocation.raise_()
        self.SubprocessID.raise_()
        self.subID.raise_()
        self.OperationType.raise_()
        self.opType.raise_()
        self.SubprocessStartTime.raise_()
        self.subStartTime.raise_()
        self.SubprocessEndTime.raise_()
        self.subEndTime.raise_()
        self.addToList.raise_()
        self.JobStartTime.raise_()
        self.startTime.raise_()
        self.JobEndTime.raise_()
        self.endTime.raise_()
        self.JobPriority.raise_()
        self.priority.raise_()
        self.addJobButton.raise_()
        self.ActiveJobList.raise_()
        self.SubprocessList.raise_()
        self.jobListWidget.raise_()
        self.JobDeleteButton.raise_()
        self.subprocessListWidget_1.raise_()
        self.subprocessListWidget_2.raise_()
        self.statusbar = QStatusBar(MainWindow)
        self.statusbar.setObjectName(u"statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)

        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"MainWindow", None))
        self.stopButton.setText(QCoreApplication.translate("MainWindow", u"Stop", None))
        self.JobID.setText(QCoreApplication.translate("MainWindow", u"Job ID", None))
        self.PartID.setText(QCoreApplication.translate("MainWindow", u"Part ID", None))
        self.PartLocation.setText(QCoreApplication.translate("MainWindow", u"Part Location", None))
        self.SubprocessID.setText(QCoreApplication.translate("MainWindow", u"Subprocess ID", None))
        self.subID.setText("")
        self.OperationType.setText(QCoreApplication.translate("MainWindow", u"Operation Type", None))
        self.SubprocessStartTime.setText(QCoreApplication.translate("MainWindow", u"Subprocess Start time", None))
        self.SubprocessEndTime.setText(QCoreApplication.translate("MainWindow", u"Subprocess End Time", None))
        self.addToList.setText(QCoreApplication.translate("MainWindow", u"Add to list", None))
        self.JobStartTime.setText(QCoreApplication.translate("MainWindow", u"Job Start Time", None))
        self.JobEndTime.setText(QCoreApplication.translate("MainWindow", u"Job End Time", None))
        self.JobPriority.setText(QCoreApplication.translate("MainWindow", u"Job Priority", None))
        self.priority.setItemText(0, QCoreApplication.translate("MainWindow", u"1", None))
        self.priority.setItemText(1, QCoreApplication.translate("MainWindow", u"2", None))
        self.priority.setItemText(2, QCoreApplication.translate("MainWindow", u"3", None))

        self.addJobButton.setText(QCoreApplication.translate("MainWindow", u"Add job", None))
        self.ActiveJobList.setText(QCoreApplication.translate("MainWindow", u"Active Job List", None))
        self.SubprocessList.setText(QCoreApplication.translate("MainWindow", u"Subprocess List", None))
        self.JobDeleteButton.setText(QCoreApplication.translate("MainWindow", u"Delete selected job", None))
    # retranslateUi

