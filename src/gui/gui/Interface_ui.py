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
        MainWindow.resize(752, 695)
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.stopButton = QPushButton(self.centralwidget)
        self.stopButton.setObjectName(u"stopButton")
        self.stopButton.setGeometry(QRect(165, 15, 500, 25))
        self.stopButton.setStyleSheet(u"\"background-color: rgb(255, 0, 0)\"")
        self.stopButton.setAutoRepeat(False)
        self.JobID = QLabel(self.centralwidget)
        self.JobID.setObjectName(u"JobID")
        self.JobID.setGeometry(QRect(9, 84, 44, 17))
        self.job_id = QLineEdit(self.centralwidget)
        self.job_id.setObjectName(u"job_id")
        self.job_id.setGeometry(QRect(165, 84, 500, 25))
        self.PartID = QLabel(self.centralwidget)
        self.PartID.setObjectName(u"PartID")
        self.PartID.setGeometry(QRect(9, 127, 47, 17))
        self.PartIDList = QLabel(self.centralwidget)
        self.PartIDList.setObjectName(u"PartIDList")
        self.PartIDList.setGeometry(QRect(9, 127, 47, 17))
        self.PartListNames = QLabel(self.centralwidget)
        self.PartListNames.setObjectName(u"PartListNames")
        self.PartListNames.setGeometry(QRect(9, 127, 47, 17))
        self.part_id = QLineEdit(self.centralwidget)
        self.part_id.setObjectName(u"part_id")
        self.part_id.setGeometry(QRect(165, 127, 500, 25))
        self.PartLocation = QLabel(self.centralwidget)
        self.PartLocation.setObjectName(u"PartLocation")
        self.PartLocation.setGeometry(QRect(9, 164, 91, 17))
        self.partLocation = QLineEdit(self.centralwidget)
        self.partLocation.setObjectName(u"partLocation")
        self.partLocation.setGeometry(QRect(165, 164, 500, 25))
        self.SubprocessID = QLabel(self.centralwidget)
        self.SubprocessID.setObjectName(u"SubprocessID")
        self.SubprocessID.setGeometry(QRect(20, 209, 96, 17))
        self.subID = QLineEdit(self.centralwidget)
        self.subID.setObjectName(u"subID")
        self.subID.setGeometry(QRect(165, 209, 500, 25))
        self.OperationType = QLabel(self.centralwidget)
        self.OperationType.setObjectName(u"OperationType")
        self.OperationType.setGeometry(QRect(100, 240, 106, 17))
        self.opType = QLineEdit(self.centralwidget)
        self.opType.setObjectName(u"opType")
        self.opType.setGeometry(QRect(265, 240, 400, 25))
        self.SubprocessStartTime = QLabel(self.centralwidget)
        self.SubprocessStartTime.setObjectName(u"SubprocessStartTime")
        self.SubprocessStartTime.setGeometry(QRect(100, 271, 150, 17))
        self.subStartTime = QLineEdit(self.centralwidget)
        self.subStartTime.setObjectName(u"subStartTime")
        self.subStartTime.setGeometry(QRect(265, 271, 400, 25))
        self.SubprocessEndTime = QLabel(self.centralwidget)
        self.SubprocessEndTime.setObjectName(u"SubprocessEndTime")
        self.SubprocessEndTime.setGeometry(QRect(100, 302, 145, 17))
        self.subEndTime = QLineEdit(self.centralwidget)
        self.subEndTime.setObjectName(u"subEndTime")
        self.subEndTime.setGeometry(QRect(265, 302, 400, 25))
        self.addToList = QPushButton(self.centralwidget)
        self.addToList.setObjectName(u"addToList")
        self.addToList.setGeometry(QRect(165, 333, 500, 25))
        self.JobStartTime = QLabel(self.centralwidget)
        self.JobStartTime.setObjectName(u"JobStartTime")
        self.JobStartTime.setGeometry(QRect(9, 364, 100, 17))
        self.startTime = QLineEdit(self.centralwidget)
        self.startTime.setObjectName(u"startTime")
        self.startTime.setGeometry(QRect(165, 364, 142, 25))
        self.JobEndTime = QLabel(self.centralwidget)
        self.JobEndTime.setObjectName(u"JobEndTime")
        self.JobEndTime.setGeometry(QRect(9, 407, 92, 17))
        self.endTime = QLineEdit(self.centralwidget)
        self.endTime.setObjectName(u"endTime")
        self.endTime.setGeometry(QRect(165, 407, 142, 25))
        self.JobPriority = QLabel(self.centralwidget)
        self.JobPriority.setObjectName(u"JobPriority")
        self.JobPriority.setGeometry(QRect(9, 450, 81, 17))
        self.priority = QComboBox(self.centralwidget)
        self.priority.addItem("")
        self.priority.addItem("")
        self.priority.addItem("")
        self.priority.setObjectName(u"priority")
        self.priority.setGeometry(QRect(165, 450, 38, 25))
        self.addJobButton = QPushButton(self.centralwidget)
        self.addJobButton.setObjectName(u"addJobButton")
        self.addJobButton.setGeometry(QRect(165, 481, 80, 25))
        self.ActiveJobList = QLabel(self.centralwidget)
        self.ActiveJobList.setObjectName(u"ActiveJobList")
        self.ActiveJobList.setGeometry(QRect(165, 538, 99, 17))
        self.JobListWidget = QListWidget(self.centralwidget)
        self.JobListWidget.setObjectName(u"JobListWidget")
        self.JobListWidget.setGeometry(QRect(165, 560, 250, 72))
        self.JobDeleteButton = QPushButton(self.centralwidget)
        self.JobDeleteButton.setObjectName(u"JobDeleteButton")
        self.JobDeleteButton.setGeometry(QRect(165, 639, 140, 25))
        self.PartSelectButton = QPushButton(self.centralwidget)
        self.PartSelectButton.setObjectName(u"PartSelectButton")
        self.PartSelectButton.setGeometry(QRect(165, 639, 140, 25))
        self.listWidget_1 = QListWidget(self.centralwidget)
        self.listWidget_1.setObjectName(u"listWidget_1")
        self.listWidget_1.setGeometry(QRect(420, 560, 250, 72))
        self.listWidget_2 = QListWidget(self.centralwidget)
        self.listWidget_2.setObjectName(u"listWidget_2")
        self.listWidget_2.setGeometry(QRect(420, 560, 250, 72))
        self.listWidget_3 = QListWidget(self.centralwidget)
        self.listWidget_3.setObjectName(u"listWidget_3")
        self.listWidget_3.setGeometry(QRect(0, 0, 250, 72))
        self.listWidget_4 = QListWidget(self.centralwidget)
        self.listWidget_4.setObjectName(u"listWidget_4")
        self.listWidget_4.setGeometry(QRect(0, 0, 250, 72))
        self.frame = QFrame(self.centralwidget)
        self.frame.setObjectName(u"frame")
        self.frame.setGeometry(QRect(10, 200, 661, 161))
        self.frame.setAutoFillBackground(False)
        self.frame.setFrameShape(QFrame.StyledPanel)
        self.frame.setFrameShadow(QFrame.Raised)
        self.frame.setLineWidth(3)
        self.frame.setMidLineWidth(3)
        self.widget = QWidget(self.centralwidget)
        self.widget.setObjectName(u"widget")
        self.widget.setGeometry(QRect(0, 0, 120, 80))
        self.widget1 = QWidget(self.centralwidget)
        self.widget1.setObjectName(u"widget1")
        self.widget1.setGeometry(QRect(30, 20, 16, 16))
        self.formLayout = QFormLayout(self.widget1)
        self.formLayout.setObjectName(u"formLayout")
        self.formLayout.setContentsMargins(0, 0, 0, 0)
        MainWindow.setCentralWidget(self.centralwidget)
        self.frame.raise_()
        self.stopButton.raise_()
        self.JobID.raise_()
        self.job_id.raise_()
        self.PartID.raise_()
        self.PartIDList.raise_()
        self.PartListNames.raise_()
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
        self.JobListWidget.raise_()
        self.JobDeleteButton.raise_()
        self.PartSelectButton.raise_()
        self.listWidget_1.raise_()
        self.listWidget_2.raise_()
        self.listWidget_3.raise_()
        self.listWidget_4.raise_()
        self.widget.raise_()
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
        self.PartIDList.setText(QCoreApplication.translate("MainWindow", u"Part ID", None))
        self.PartListNames.setText(QCoreApplication.translate("MainWindow", u"Part ID", None))
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

        self.addJobButton.setText(QCoreApplication.translate("MainWindow", u"add job", None))
        self.ActiveJobList.setText(QCoreApplication.translate("MainWindow", u"Active Job List", None))
        self.JobDeleteButton.setText(QCoreApplication.translate("MainWindow", u"Delete selected job", None))
        self.PartSelectButton.setText(QCoreApplication.translate("MainWindow", u"Select Part", None))
    # retranslateUi

