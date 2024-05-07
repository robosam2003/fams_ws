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
        

        yDefaultSpacing = 40
        xLabel = 20
        yJobFrame = 60
        yJobId = yJobFrame + 10
        xLineEdit = 165
        yPartId = yJobId + yDefaultSpacing
        yPartLists = yPartId + 20
        hPartIdListWidget = 70
        yPartSelect = yPartLists + hPartIdListWidget + 10
        yPartLocation = yPartSelect + yDefaultSpacing
        ySubFrame = yPartLocation + yDefaultSpacing
        ySubId = ySubFrame + 10
        xSubIndent = 100
        xLineEditSub = xLineEdit + xSubIndent
        ySubOpType = ySubId + yDefaultSpacing
        ySubStart = ySubOpType + yDefaultSpacing
        ySubEnd = ySubStart + yDefaultSpacing
        yAddSubToList = ySubEnd + yDefaultSpacing
        hSubFrame = yAddSubToList + yDefaultSpacing - ySubFrame
        yJobStart = ySubFrame + hSubFrame + 10
        yJobEnd = yJobStart + yDefaultSpacing
        yJobPriority = yJobEnd + yDefaultSpacing
        yAddJob = yJobPriority + yDefaultSpacing
        wJobFrame = 665
        hJobFrame = yAddJob + yDefaultSpacing - yJobFrame
        yLowerListWidgetLabels = yJobFrame + hJobFrame + 20
        yLowerListWidgets = yLowerListWidgetLabels + 20
        hLowerListWidgets = 70
        yDeleteJob = yLowerListWidgets + hLowerListWidgets + 10
        wJobWidget=235
        wSubWidget=200
        wPartWidget = 245
        xSubWidget1=10+wJobWidget+xLabel
        xSubWidget2=xSubWidget1+wSubWidget+10
        xPartIdList = xLineEdit
        xPartListNames = 10 + wJobFrame - wPartWidget - 10
        xRightAlign = 10 + wJobFrame + 10

        mainWindowSize=[850, yDeleteJob + 2*yDefaultSpacing]
        MainWindow.resize(mainWindowSize[0],mainWindowSize[1])

        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.stopButton = QPushButton(self.centralwidget)
        self.stopButton.setObjectName(u"stopButton")
        self.stopButton.setGeometry(QRect(165, 15, 500, 25))    # xLocation, yLocation, Width, Height (From top left)
        self.stopButton.setAutoRepeat(False)
        self.stopButton.setStyleSheet("background-color: rgb(255, 0, 0)")
        
        self.JobID = QLabel(self.centralwidget)
        self.JobID.setObjectName(u"JobID")
        self.JobID.setGeometry(QRect(xLabel, yJobId, 44, 17))
        self.job_id = QLineEdit(self.centralwidget)
        self.job_id.setObjectName(u"job_id")
        self.job_id.setGeometry(QRect(xLineEdit, yJobId, 500, 25))
        
        self.PartID = QLabel(self.centralwidget)
        self.PartID.setObjectName(u"PartID")
        self.PartID.setGeometry(QRect(xLabel, yPartId, 91, 17))
        self.part_id = QLineEdit(self.centralwidget)
        self.part_id.setObjectName(u"part_id")
        self.part_id.setGeometry(QRect(xLineEdit, 0, 500, 25))
        self.PartIDList = QLabel(self.centralwidget)
        self.PartIDList.setObjectName(u"PartIDList")
        self.PartIDList.setGeometry(QRect(xPartIdList, yPartId, 91, 17))
        self.PartListNames = QLabel(self.centralwidget)
        self.PartListNames.setObjectName(u"PartListNames")
        self.PartListNames.setGeometry(QRect(xPartListNames, yPartId, 91, 17))
        
        self.PartLocation = QLabel(self.centralwidget)
        self.PartLocation.setObjectName(u"PartLocation")
        self.PartLocation.setGeometry(QRect(xLabel, yPartLocation, 91, 17))
        self.partLocation = QLineEdit(self.centralwidget)
        self.partLocation.setObjectName(u"partLocation")
        self.partLocation.setGeometry(QRect(xLineEdit, yPartLocation, 500, 25))
        
        self.SubprocessID = QLabel(self.centralwidget)
        self.SubprocessID.setObjectName(u"SubprocessID")
        self.SubprocessID.setGeometry(QRect(xLabel+10, ySubId, 96, 17))
        self.subID = QLineEdit(self.centralwidget)
        self.subID.setObjectName(u"subID")
        self.subID.setGeometry(QRect(xLineEdit, ySubId, 480, 25))
        
        self.OperationType = QLabel(self.centralwidget)
        self.OperationType.setObjectName(u"OperationType")
        self.OperationType.setGeometry(QRect(xSubIndent, ySubOpType, 106, 17))
        self.opType = QLineEdit(self.centralwidget)
        self.opType.setObjectName(u"opType")
        self.opType.setGeometry(QRect(xLineEditSub, ySubOpType, 380, 25))

        self.SubprocessStartTime = QLabel(self.centralwidget)
        self.SubprocessStartTime.setObjectName(u"SubprocessStartTime")
        self.SubprocessStartTime.setGeometry(QRect(xSubIndent, ySubStart, 150, 17))
        self.subStartTime = QLineEdit(self.centralwidget)
        self.subStartTime.setObjectName(u"subStartTime")
        self.subStartTime.setGeometry(QRect(xLineEditSub, ySubStart, 380, 25))

        self.SubprocessEndTime = QLabel(self.centralwidget)
        self.SubprocessEndTime.setObjectName(u"SubprocessEndTime")
        self.SubprocessEndTime.setGeometry(QRect(xSubIndent, ySubEnd, 145, 17))
        self.subEndTime = QLineEdit(self.centralwidget)
        self.subEndTime.setObjectName(u"subEndTime")
        self.subEndTime.setGeometry(QRect(xLineEditSub, ySubEnd, 380, 25))
        
        self.addToList = QPushButton(self.centralwidget)
        self.addToList.setObjectName(u"addToList")
        self.addToList.setGeometry(QRect(xLineEdit, yAddSubToList, 480, 25))
        self.addToList.setStyleSheet("background-color: rgb(70, 100, 200)")
        self.JobStartTime = QLabel(self.centralwidget)
        self.JobStartTime.setObjectName(u"JobStartTime")
        self.JobStartTime.setGeometry(QRect(xLabel, yJobStart, 100, 17))
        self.startTime = QLineEdit(self.centralwidget)
        self.startTime.setObjectName(u"startTime")
        self.startTime.setGeometry(QRect(xLineEdit, yJobStart, 500, 25))
        self.JobEndTime = QLabel(self.centralwidget)
        self.JobEndTime.setObjectName(u"JobEndTime")
        self.JobEndTime.setGeometry(QRect(xLabel, yJobEnd, 92, 17))
        self.endTime = QLineEdit(self.centralwidget)
        self.endTime.setObjectName(u"endTime")
        self.endTime.setGeometry(QRect(xLineEdit, yJobEnd, 500, 25))
        self.JobPriority = QLabel(self.centralwidget)
        self.JobPriority.setObjectName(u"JobPriority")
        self.JobPriority.setGeometry(QRect(xLabel, yJobPriority, 81, 17))
        self.priority = QComboBox(self.centralwidget)
        self.priority.addItem("")
        self.priority.addItem("")
        self.priority.addItem("")
        self.priority.setObjectName(u"priority")
        self.priority.setGeometry(QRect(xLineEdit, yJobPriority, 38, 25))
        self.addJobButton = QPushButton(self.centralwidget)
        self.addJobButton.setObjectName(u"addJobButton")
        self.addJobButton.setGeometry(QRect(xLineEdit, yAddJob, 500, 25))
        self.addJobButton.setStyleSheet("background-color: rgb(60, 60, 80)")
        self.ActiveJobList = QLabel(self.centralwidget)
        self.ActiveJobList.setObjectName(u"ActiveJobList")
        self.ActiveJobList.setGeometry(QRect(10, yLowerListWidgetLabels, 99, 17))

        
        self.jobListWidget = QListWidget(self.centralwidget)
        self.jobListWidget.setObjectName(u"JobListWidget")
        self.jobListWidget.setGeometry(QRect(10, yLowerListWidgets, wJobWidget, hLowerListWidgets))
        self.JobDeleteButton = QPushButton(self.centralwidget)
        self.JobDeleteButton.setObjectName(u"JobDeleteButton")
        self.JobDeleteButton.setGeometry(QRect(10, yDeleteJob, wJobWidget, 25))
        self.ClearSubprocessesButton = QPushButton(self.centralwidget)
        self.ClearSubprocessesButton.setObjectName(u"ClearSubprocessesButton")
        self.ClearSubprocessesButton.setGeometry(QRect(xSubWidget1, yDeleteJob, wSubWidget + 10 + wSubWidget, 25))
        self.PartSelectButton = QPushButton(self.centralwidget)
        self.PartSelectButton.setObjectName(u"PartSelectButton")
        self.PartSelectButton.setGeometry(QRect(xLineEdit, yPartSelect, 10 + wJobFrame - 10 - xLineEdit, 25))
        self.LoadedButton = QPushButton(self.centralwidget)
        self.LoadedButton.setObjectName(u"LoadedButton")
        self.LoadedButton.setGeometry(QRect(xRightAlign, yPartLists, 140, 25))
        self.UnloadedButton = QPushButton(self.centralwidget)
        self.UnloadedButton.setObjectName(u"UnloadedButton")
        self.UnloadedButton.setGeometry(QRect(xRightAlign, yPartLists + yDefaultSpacing, 140, 25))
        self.LoadPresetButton = QPushButton(self.centralwidget)
        self.LoadPresetButton.setObjectName(u"LoadPresetButton")
        self.LoadPresetButton.setGeometry(QRect(xRightAlign, 15, 140, 25))

        self.LayoutButton1 = QPushButton(self.centralwidget)
        self.LayoutButton1.setObjectName(u"LayoutButton1")
        self.LayoutButton1.setGeometry(QRect(xRightAlign + 140 - 25 - 35, yDeleteJob, 25, 25))
        self.LayoutButton2 = QPushButton(self.centralwidget)
        self.LayoutButton2.setObjectName(u"LayoutButton2")
        self.LayoutButton2.setGeometry(QRect(xRightAlign + 140 - 25, yDeleteJob, 25, 25))

        self.SubprocessList = QLabel(self.centralwidget)
        self.SubprocessList.setObjectName(u"SubprocessList")
        self.SubprocessList.setGeometry(QRect(xSubWidget1, yLowerListWidgetLabels, 110, 17))
        
        self.subprocessListWidget_1 = QListWidget(self.centralwidget)
        self.subprocessListWidget_1.setObjectName(u"listWidget_1")
        self.subprocessListWidget_1.setGeometry(QRect(xSubWidget1, yLowerListWidgets, wSubWidget, hLowerListWidgets))
        self.subprocessListWidget_2 = QListWidget(self.centralwidget)
        self.subprocessListWidget_2.setObjectName(u"listWidget_2")
        self.subprocessListWidget_2.setGeometry(QRect(xSubWidget2, yLowerListWidgets, wSubWidget, hLowerListWidgets))

        self.subprocessListWidget_3 = QListWidget(self.centralwidget) #PartListWidget1
        self.subprocessListWidget_3.setObjectName(u"listWidget_3")
        self.subprocessListWidget_3.setGeometry(QRect(xLineEdit, yPartLists, wPartWidget, hPartIdListWidget))
        self.subprocessListWidget_4 = QListWidget(self.centralwidget) #PartListWidget2
        self.subprocessListWidget_4.setObjectName(u"listWidget_4")
        self.subprocessListWidget_4.setGeometry(QRect(xPartListNames, yPartLists, wPartWidget, hPartIdListWidget))

        
        self.frame = QFrame(self.centralwidget)     # Subprocess Frame
        self.frame.setObjectName(u"frame")
        self.frame.setGeometry(QRect(xLabel, ySubFrame, 645, hSubFrame))
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
        self.frame1.setGeometry(QRect(10, yJobFrame, wJobFrame, hJobFrame))
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
        #self.part_id.raise_()
        self.PartIDList.raise_()
        self.PartListNames.raise_()
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
        self.ClearSubprocessesButton.raise_()
        self.PartSelectButton.raise_()
        self.LoadedButton.raise_()
        self.UnloadedButton.raise_()
        self.LoadPresetButton.raise_()
        #self.LayoutButton1.raise_()
        #self.LayoutButton2.raise_()
        self.subprocessListWidget_1.raise_()
        self.subprocessListWidget_2.raise_()
        self.subprocessListWidget_3.raise_()
        self.subprocessListWidget_4.raise_()
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
        self.PartID.setText(QCoreApplication.translate("MainWindow", u"Parts List", None))
        self.PartIDList.setText(QCoreApplication.translate("MainWindow", u"Part ID List", None))
        self.PartListNames.setText(QCoreApplication.translate("MainWindow", u"Parts Names", None))
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
        self.ClearSubprocessesButton.setText(QCoreApplication.translate("MainWindow", u"Clear subprocess list", None))
        self.PartSelectButton.setText(QCoreApplication.translate("MainWindow", u"Select Part", None))
        self.LoadedButton.setText(QCoreApplication.translate("MainWindow", u"Loaded", None))
        self.UnloadedButton.setText(QCoreApplication.translate("MainWindow", u"Unloaded", None))
        self.LoadPresetButton.setText(QCoreApplication.translate("MainWindow", u"Load Preset", None))
        self.LayoutButton1.setText("")
        self.LayoutButton2.setText("")
    # retranslateUi

