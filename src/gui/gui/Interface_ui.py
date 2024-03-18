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
    QLineEdit, QListView, QMainWindow, QPushButton,
    QSizePolicy, QSpacerItem, QStatusBar, QWidget)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(701, 673)
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.formLayout_2 = QFormLayout(self.centralwidget)
        self.formLayout_2.setObjectName(u"formLayout_2")
        self.stopButton = QPushButton(self.centralwidget)
        self.stopButton.setObjectName(u"stopButton")
        icon = QIcon(QIcon.fromTheme(u"media-playback-stop"))
        self.stopButton.setIcon(icon)
        self.stopButton.setAutoRepeat(False)

        self.formLayout_2.setWidget(1, QFormLayout.FieldRole, self.stopButton)

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

        self.label10 = QLabel(self.centralwidget)
        self.label10.setObjectName(u"label10")

        self.formLayout_2.setWidget(10, QFormLayout.LabelRole, self.label10)

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

        self.listView = QListView(self.centralwidget)
        self.listView.setObjectName(u"listView")

        self.formLayout_2.setWidget(27, QFormLayout.FieldRole, self.listView)

        self.horizontalSpacer = QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.formLayout_2.setItem(26, QFormLayout.FieldRole, self.horizontalSpacer)

        self.horizontalSpacer_2 = QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.formLayout_2.setItem(3, QFormLayout.FieldRole, self.horizontalSpacer_2)

        MainWindow.setCentralWidget(self.centralwidget)
        self.statusbar = QStatusBar(MainWindow)
        self.statusbar.setObjectName(u"statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)

        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"MainWindow", None))
        self.stopButton.setText(QCoreApplication.translate("MainWindow", u"stop", None))
        self.label.setText(QCoreApplication.translate("MainWindow", u"job id", None))
        self.label_2.setText(QCoreApplication.translate("MainWindow", u"part id", None))
        self.label10.setText(QCoreApplication.translate("MainWindow", u"partLocation", None))
        self.label_3.setText(QCoreApplication.translate("MainWindow", u"subprocessesID", None))
        self.subID.setText("")
        self.label_7.setText(QCoreApplication.translate("MainWindow", u"operationType", None))
        self.label_8.setText(QCoreApplication.translate("MainWindow", u"startTime", None))
        self.label_9.setText(QCoreApplication.translate("MainWindow", u"endTime", None))
        self.addToList.setText(QCoreApplication.translate("MainWindow", u"add to list", None))
        self.label_4.setText(QCoreApplication.translate("MainWindow", u"start", None))
        self.label_5.setText(QCoreApplication.translate("MainWindow", u"end", None))
        self.label_6.setText(QCoreApplication.translate("MainWindow", u"priority", None))
        self.priority.setItemText(0, QCoreApplication.translate("MainWindow", u"1", None))
        self.priority.setItemText(1, QCoreApplication.translate("MainWindow", u"2", None))
        self.priority.setItemText(2, QCoreApplication.translate("MainWindow", u"3", None))

        self.addJobButton.setText(QCoreApplication.translate("MainWindow", u"add job", None))
    # retranslateUi

