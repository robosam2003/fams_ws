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
from PySide6.QtWidgets import (QApplication, QComboBox, QFormLayout, QLabel,
    QLineEdit, QListWidget, QListWidgetItem, QMainWindow,
    QPushButton, QSizePolicy, QSpacerItem, QStatusBar,
    QWidget)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(720, 695)
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.formLayout_2 = QFormLayout(self.centralwidget)
        self.formLayout_2.setObjectName(u"formLayout_2")
        self.stopButton = QPushButton(self.centralwidget)
        self.stopButton.setObjectName(u"stopButton")
        self.stopButton.setStyleSheet("background-color: red")
        self.stopButton.setAutoRepeat(False)

        self.formLayout_2.setWidget(1, QFormLayout.FieldRole, self.stopButton)

        self.horizontalSpacer_2 = QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.formLayout_2.setItem(3, QFormLayout.FieldRole, self.horizontalSpacer_2)

        self.label = QLabel(self.centralwidget)
        self.label.setObjectName(u"label")

        self.formLayout_2.setWidget(5, QFormLayout.LabelRole, self.label)

        self.job_id = QLineEdit(self.centralwidget)
        self.job_id.setObjectName(u"job_id")

        self.formLayout_2.setWidget(5, QFormLayout.FieldRole, self.job_id)

        self.label_2 = QLabel(self.centralwidget)
        self.label_2.setObjectName(u"label_2")

        self.formLayout_2.setWidget(8, QFormLayout.LabelRole, self.label_2)

        self.part_id = QLineEdit(self.centralwidget)
        self.part_id.setObjectName(u"part_id")

        self.formLayout_2.setWidget(8, QFormLayout.FieldRole, self.part_id)

        self.label_11 = QLabel(self.centralwidget)
        self.label_11.setObjectName(u"label_11")

        self.formLayout_2.setWidget(10, QFormLayout.LabelRole, self.label_11)

        self.partLocation = QLineEdit(self.centralwidget)
        self.partLocation.setObjectName(u"partLocation")

        self.formLayout_2.setWidget(10, QFormLayout.FieldRole, self.partLocation)

        self.formLayout = QFormLayout()
        self.formLayout.setObjectName(u"formLayout")

        self.formLayout_2.setLayout(12, QFormLayout.SpanningRole, self.formLayout)

        self.label_3 = QLabel(self.centralwidget)
        self.label_3.setObjectName(u"label_3")

        self.formLayout_2.setWidget(13, QFormLayout.LabelRole, self.label_3)

        self.subID = QLineEdit(self.centralwidget)
        self.subID.setObjectName(u"subID")

        self.formLayout_2.setWidget(13, QFormLayout.FieldRole, self.subID)

        self.label_7 = QLabel(self.centralwidget)
        self.label_7.setObjectName(u"label_7")

        self.formLayout_2.setWidget(14, QFormLayout.LabelRole, self.label_7)

        self.opType = QLineEdit(self.centralwidget)
        self.opType.setObjectName(u"opType")

        self.formLayout_2.setWidget(14, QFormLayout.FieldRole, self.opType)

        self.label_8 = QLabel(self.centralwidget)
        self.label_8.setObjectName(u"label_8")

        self.formLayout_2.setWidget(15, QFormLayout.LabelRole, self.label_8)

        self.subStartTime = QLineEdit(self.centralwidget)
        self.subStartTime.setObjectName(u"subStartTime")

        self.formLayout_2.setWidget(15, QFormLayout.FieldRole, self.subStartTime)

        self.label_9 = QLabel(self.centralwidget)
        self.label_9.setObjectName(u"label_9")

        self.formLayout_2.setWidget(16, QFormLayout.LabelRole, self.label_9)

        self.subEndTime = QLineEdit(self.centralwidget)
        self.subEndTime.setObjectName(u"subEndTime")

        self.formLayout_2.setWidget(16, QFormLayout.FieldRole, self.subEndTime)

        self.addToList = QPushButton(self.centralwidget)
        self.addToList.setObjectName(u"addToList")

        self.formLayout_2.setWidget(17, QFormLayout.FieldRole, self.addToList)

        self.label_4 = QLabel(self.centralwidget)
        self.label_4.setObjectName(u"label_4")

        self.formLayout_2.setWidget(18, QFormLayout.LabelRole, self.label_4)

        self.startTime = QLineEdit(self.centralwidget)
        self.startTime.setObjectName(u"startTime")

        self.formLayout_2.setWidget(18, QFormLayout.FieldRole, self.startTime)

        self.label_5 = QLabel(self.centralwidget)
        self.label_5.setObjectName(u"label_5")

        self.formLayout_2.setWidget(21, QFormLayout.LabelRole, self.label_5)

        self.endTime = QLineEdit(self.centralwidget)
        self.endTime.setObjectName(u"endTime")

        self.formLayout_2.setWidget(21, QFormLayout.FieldRole, self.endTime)

        self.label_6 = QLabel(self.centralwidget)
        self.label_6.setObjectName(u"label_6")

        self.formLayout_2.setWidget(24, QFormLayout.LabelRole, self.label_6)

        self.priority = QComboBox(self.centralwidget)
        self.priority.addItem("")
        self.priority.addItem("")
        self.priority.addItem("")
        self.priority.setObjectName(u"priority")

        self.formLayout_2.setWidget(24, QFormLayout.FieldRole, self.priority)

        self.addJobButton = QPushButton(self.centralwidget)
        self.addJobButton.setObjectName(u"addJobButton")

        self.formLayout_2.setWidget(25, QFormLayout.FieldRole, self.addJobButton)

        self.horizontalSpacer = QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.formLayout_2.setItem(26, QFormLayout.FieldRole, self.horizontalSpacer)

        self.label_10 = QLabel(self.centralwidget)
        self.label_10.setObjectName(u"label_10")

        self.formLayout_2.setWidget(27, QFormLayout.FieldRole, self.label_10)

        self.listWidget = QListWidget(self.centralwidget)
        self.listWidget.setObjectName(u"listWidget")

        self.formLayout_2.setWidget(28, QFormLayout.FieldRole, self.listWidget)

        self.JobDeleteButton = QPushButton(self.centralwidget)
        self.JobDeleteButton.setObjectName(u"JobDeleteButton")

        self.formLayout_2.setWidget(29, QFormLayout.FieldRole, self.JobDeleteButton)

        MainWindow.setCentralWidget(self.centralwidget)
        self.statusbar = QStatusBar(MainWindow)
        self.statusbar.setObjectName(u"statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)

        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"MainWindow", None))
        self.stopButton.setText(QCoreApplication.translate("MainWindow", u"Stop", None))
        self.label.setText(QCoreApplication.translate("MainWindow", u"Job ID", None))
        self.label_2.setText(QCoreApplication.translate("MainWindow", u"Part ID", None))
        self.label_11.setText(QCoreApplication.translate("MainWindow", u"Part Location", None))
        self.label_3.setText(QCoreApplication.translate("MainWindow", u"Subprocess ID", None))
        self.subID.setText("")
        self.label_7.setText(QCoreApplication.translate("MainWindow", u"Operation Type", None))
        self.label_8.setText(QCoreApplication.translate("MainWindow", u"Subprocess Start time", None))
        self.label_9.setText(QCoreApplication.translate("MainWindow", u"Subprocess End Time", None))
        self.addToList.setText(QCoreApplication.translate("MainWindow", u"add to list", None))
        self.label_4.setText(QCoreApplication.translate("MainWindow", u"Job Start Time", None))
        self.label_5.setText(QCoreApplication.translate("MainWindow", u"Job End Time", None))
        self.label_6.setText(QCoreApplication.translate("MainWindow", u"Job Priority", None))
        self.priority.setItemText(0, QCoreApplication.translate("MainWindow", u"1", None))
        self.priority.setItemText(1, QCoreApplication.translate("MainWindow", u"2", None))
        self.priority.setItemText(2, QCoreApplication.translate("MainWindow", u"3", None))

        self.addJobButton.setText(QCoreApplication.translate("MainWindow", u"add job", None))
        self.label_10.setText(QCoreApplication.translate("MainWindow", u"Job List", None))
        self.JobDeleteButton.setText(QCoreApplication.translate("MainWindow", u"Delete selected job", None))
    # retranslateUi

