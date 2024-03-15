# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'Controller.ui'
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
from PySide6.QtWidgets import (QApplication, QGridLayout, QLabel, QMainWindow,
    QPushButton, QSizePolicy, QStatusBar, QWidget)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(800, 600)
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.gridLayout_2 = QGridLayout(self.centralwidget)
        self.gridLayout_2.setObjectName(u"gridLayout_2")
        self.gridLayout = QGridLayout()
        self.gridLayout.setObjectName(u"gridLayout")
        self.axis3DownButton = QPushButton(self.centralwidget)
        self.axis3DownButton.setObjectName(u"axis3DownButton")

        self.gridLayout.addWidget(self.axis3DownButton, 3, 2, 1, 1)

        self.axis4Label = QLabel(self.centralwidget)
        self.axis4Label.setObjectName(u"axis4Label")
        self.axis4Label.setMaximumSize(QSize(100, 100))

        self.gridLayout.addWidget(self.axis4Label, 0, 3, 1, 1)

        self.axis2UpButton = QPushButton(self.centralwidget)
        self.axis2UpButton.setObjectName(u"axis2UpButton")

        self.gridLayout.addWidget(self.axis2UpButton, 1, 1, 1, 1)

        self.axis2Label = QLabel(self.centralwidget)
        self.axis2Label.setObjectName(u"axis2Label")
        self.axis2Label.setMaximumSize(QSize(100, 100))

        self.gridLayout.addWidget(self.axis2Label, 0, 1, 1, 1)

        self.axis4UpButton = QPushButton(self.centralwidget)
        self.axis4UpButton.setObjectName(u"axis4UpButton")

        self.gridLayout.addWidget(self.axis4UpButton, 1, 3, 1, 1)

        self.axis3Label = QLabel(self.centralwidget)
        self.axis3Label.setObjectName(u"axis3Label")
        self.axis3Label.setMaximumSize(QSize(100, 100))

        self.gridLayout.addWidget(self.axis3Label, 0, 2, 1, 1)

        self.zeroAxis6Button = QPushButton(self.centralwidget)
        self.zeroAxis6Button.setObjectName(u"zeroAxis6Button")

        self.gridLayout.addWidget(self.zeroAxis6Button, 4, 5, 1, 1)

        self.axis6UpButton = QPushButton(self.centralwidget)
        self.axis6UpButton.setObjectName(u"axis6UpButton")

        self.gridLayout.addWidget(self.axis6UpButton, 1, 5, 1, 1)

        self.axis4PositionLabel = QLabel(self.centralwidget)
        self.axis4PositionLabel.setObjectName(u"axis4PositionLabel")
        self.axis4PositionLabel.setMaximumSize(QSize(100, 100))

        self.gridLayout.addWidget(self.axis4PositionLabel, 2, 3, 1, 1)

        self.axis1Label = QLabel(self.centralwidget)
        self.axis1Label.setObjectName(u"axis1Label")
        sizePolicy = QSizePolicy(QSizePolicy.Minimum, QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.axis1Label.sizePolicy().hasHeightForWidth())
        self.axis1Label.setSizePolicy(sizePolicy)
        self.axis1Label.setMaximumSize(QSize(100, 100))

        self.gridLayout.addWidget(self.axis1Label, 0, 0, 1, 1)

        self.axis1PositionLabel = QLabel(self.centralwidget)
        self.axis1PositionLabel.setObjectName(u"axis1PositionLabel")
        sizePolicy.setHeightForWidth(self.axis1PositionLabel.sizePolicy().hasHeightForWidth())
        self.axis1PositionLabel.setSizePolicy(sizePolicy)
        self.axis1PositionLabel.setMaximumSize(QSize(100, 100))

        self.gridLayout.addWidget(self.axis1PositionLabel, 2, 0, 1, 1)

        self.axis1UpButton = QPushButton(self.centralwidget)
        self.axis1UpButton.setObjectName(u"axis1UpButton")

        self.gridLayout.addWidget(self.axis1UpButton, 1, 0, 1, 1)

        self.axis5Label = QLabel(self.centralwidget)
        self.axis5Label.setObjectName(u"axis5Label")
        self.axis5Label.setMaximumSize(QSize(100, 100))

        self.gridLayout.addWidget(self.axis5Label, 0, 4, 1, 1)

        self.axis3PositionLabel = QLabel(self.centralwidget)
        self.axis3PositionLabel.setObjectName(u"axis3PositionLabel")
        self.axis3PositionLabel.setMaximumSize(QSize(100, 100))

        self.gridLayout.addWidget(self.axis3PositionLabel, 2, 2, 1, 1)

        self.zeroAxis5Button = QPushButton(self.centralwidget)
        self.zeroAxis5Button.setObjectName(u"zeroAxis5Button")

        self.gridLayout.addWidget(self.zeroAxis5Button, 4, 4, 1, 1)

        self.axis2DownButton = QPushButton(self.centralwidget)
        self.axis2DownButton.setObjectName(u"axis2DownButton")

        self.gridLayout.addWidget(self.axis2DownButton, 3, 1, 1, 1)

        self.axis1DownButton = QPushButton(self.centralwidget)
        self.axis1DownButton.setObjectName(u"axis1DownButton")

        self.gridLayout.addWidget(self.axis1DownButton, 3, 0, 1, 1)

        self.axis6Label = QLabel(self.centralwidget)
        self.axis6Label.setObjectName(u"axis6Label")
        self.axis6Label.setMaximumSize(QSize(100, 100))

        self.gridLayout.addWidget(self.axis6Label, 0, 5, 1, 1)

        self.axis5UpButton = QPushButton(self.centralwidget)
        self.axis5UpButton.setObjectName(u"axis5UpButton")

        self.gridLayout.addWidget(self.axis5UpButton, 1, 4, 1, 1)

        self.axis6PositionLabel = QLabel(self.centralwidget)
        self.axis6PositionLabel.setObjectName(u"axis6PositionLabel")
        self.axis6PositionLabel.setMaximumSize(QSize(100, 100))

        self.gridLayout.addWidget(self.axis6PositionLabel, 2, 5, 1, 1)

        self.axis6DownButton = QPushButton(self.centralwidget)
        self.axis6DownButton.setObjectName(u"axis6DownButton")

        self.gridLayout.addWidget(self.axis6DownButton, 3, 5, 1, 1)

        self.zeroAxis2Button = QPushButton(self.centralwidget)
        self.zeroAxis2Button.setObjectName(u"zeroAxis2Button")

        self.gridLayout.addWidget(self.zeroAxis2Button, 4, 1, 1, 1)

        self.axis5DownButton = QPushButton(self.centralwidget)
        self.axis5DownButton.setObjectName(u"axis5DownButton")

        self.gridLayout.addWidget(self.axis5DownButton, 3, 4, 1, 1)

        self.zeroAxis1Button = QPushButton(self.centralwidget)
        self.zeroAxis1Button.setObjectName(u"zeroAxis1Button")

        self.gridLayout.addWidget(self.zeroAxis1Button, 4, 0, 1, 1)

        self.axis3UpButton = QPushButton(self.centralwidget)
        self.axis3UpButton.setObjectName(u"axis3UpButton")

        self.gridLayout.addWidget(self.axis3UpButton, 1, 2, 1, 1)

        self.axis4DownButton = QPushButton(self.centralwidget)
        self.axis4DownButton.setObjectName(u"axis4DownButton")

        self.gridLayout.addWidget(self.axis4DownButton, 3, 3, 1, 1)

        self.zeroAxis4Button = QPushButton(self.centralwidget)
        self.zeroAxis4Button.setObjectName(u"zeroAxis4Button")

        self.gridLayout.addWidget(self.zeroAxis4Button, 4, 3, 1, 1)

        self.axis2PositionLabel = QLabel(self.centralwidget)
        self.axis2PositionLabel.setObjectName(u"axis2PositionLabel")
        self.axis2PositionLabel.setMaximumSize(QSize(100, 100))

        self.gridLayout.addWidget(self.axis2PositionLabel, 2, 1, 1, 1)

        self.axis5PositionLabel = QLabel(self.centralwidget)
        self.axis5PositionLabel.setObjectName(u"axis5PositionLabel")
        self.axis5PositionLabel.setMaximumSize(QSize(100, 100))

        self.gridLayout.addWidget(self.axis5PositionLabel, 2, 4, 1, 1)

        self.zeroAxis3Button = QPushButton(self.centralwidget)
        self.zeroAxis3Button.setObjectName(u"zeroAxis3Button")

        self.gridLayout.addWidget(self.zeroAxis3Button, 4, 2, 1, 1)

        self.gripperButton = QPushButton(self.centralwidget)
        self.gripperButton.setObjectName(u"gripperButton")

        self.gridLayout.addWidget(self.gripperButton, 5, 2, 1, 2)


        self.gridLayout_2.addLayout(self.gridLayout, 0, 1, 1, 1)

        MainWindow.setCentralWidget(self.centralwidget)
        self.statusbar = QStatusBar(MainWindow)
        self.statusbar.setObjectName(u"statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)

        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"MainWindow", None))
        self.axis3DownButton.setText(QCoreApplication.translate("MainWindow", u"\u2193", None))
        self.axis4Label.setText(QCoreApplication.translate("MainWindow", u"<html><head/><body><p align=\"center\"><span style=\" font-weight:700;\">Axis 4</span></p></body></html>", None))
        self.axis2UpButton.setText(QCoreApplication.translate("MainWindow", u"\u2191", None))
        self.axis2Label.setText(QCoreApplication.translate("MainWindow", u"<html><head/><body><p align=\"center\"><span style=\" font-weight:700;\">Axis 2</span></p></body></html>", None))
        self.axis4UpButton.setText(QCoreApplication.translate("MainWindow", u"\u2191", None))
        self.axis3Label.setText(QCoreApplication.translate("MainWindow", u"<html><head/><body><p align=\"center\"><span style=\" font-weight:700;\">Axis 3</span></p></body></html>", None))
        self.zeroAxis6Button.setText(QCoreApplication.translate("MainWindow", u"Zero Axis 6", None))
        self.axis6UpButton.setText(QCoreApplication.translate("MainWindow", u"\u2191", None))
        self.axis4PositionLabel.setText(QCoreApplication.translate("MainWindow", u"<html><head/><body><p align=\"center\">0 deg</p></body></html>", None))
        self.axis1Label.setText(QCoreApplication.translate("MainWindow", u"<html><head/><body><p align=\"center\"><span style=\" font-weight:700;\">Axis 1</span></p></body></html>", None))
        self.axis1PositionLabel.setText(QCoreApplication.translate("MainWindow", u"<html><head/><body><p align=\"center\">0 deg</p></body></html>", None))
        self.axis1UpButton.setText(QCoreApplication.translate("MainWindow", u"\u2191", None))
        self.axis5Label.setText(QCoreApplication.translate("MainWindow", u"<html><head/><body><p align=\"center\"><span style=\" font-weight:700;\">Axis 5</span></p></body></html>", None))
        self.axis3PositionLabel.setText(QCoreApplication.translate("MainWindow", u"<html><head/><body><p align=\"center\">0 deg</p></body></html>", None))
        self.zeroAxis5Button.setText(QCoreApplication.translate("MainWindow", u"Zero Axis 5", None))
        self.axis2DownButton.setText(QCoreApplication.translate("MainWindow", u"\u2193", None))
        self.axis1DownButton.setText(QCoreApplication.translate("MainWindow", u"\u2193", None))
        self.axis6Label.setText(QCoreApplication.translate("MainWindow", u"<html><head/><body><p align=\"center\"><span style=\" font-weight:700;\">Axis 6</span></p></body></html>", None))
        self.axis5UpButton.setText(QCoreApplication.translate("MainWindow", u"\u2191", None))
        self.axis6PositionLabel.setText(QCoreApplication.translate("MainWindow", u"<html><head/><body><p align=\"center\">0 deg</p></body></html>", None))
        self.axis6DownButton.setText(QCoreApplication.translate("MainWindow", u"\u2193", None))
        self.zeroAxis2Button.setText(QCoreApplication.translate("MainWindow", u"Zero Axis 2", None))
        self.axis5DownButton.setText(QCoreApplication.translate("MainWindow", u"\u2193", None))
        self.zeroAxis1Button.setText(QCoreApplication.translate("MainWindow", u"Zero Axis 1", None))
        self.axis3UpButton.setText(QCoreApplication.translate("MainWindow", u"\u2191", None))
        self.axis4DownButton.setText(QCoreApplication.translate("MainWindow", u"\u2193", None))
        self.zeroAxis4Button.setText(QCoreApplication.translate("MainWindow", u"Zero Axis 4", None))
        self.axis2PositionLabel.setText(QCoreApplication.translate("MainWindow", u"<html><head/><body><p align=\"center\">0 deg</p></body></html>", None))
        self.axis5PositionLabel.setText(QCoreApplication.translate("MainWindow", u"<html><head/><body><p align=\"center\">0 deg</p></body></html>", None))
        self.zeroAxis3Button.setText(QCoreApplication.translate("MainWindow", u"Zero Axis 3", None))
        self.gripperButton.setText(QCoreApplication.translate("MainWindow", u"Toggle Gripper", None))
    # retranslateUi

