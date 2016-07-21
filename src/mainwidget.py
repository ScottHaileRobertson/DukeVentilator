# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file '../QtCreatorUI/mainwidget.ui'
#
# Created: Thu Jul 21 12:32:11 2016
#      by: PyQt4 UI code generator 4.10.4
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

class Ui_MainWidget(object):
    def setupUi(self, MainWidget):
        MainWidget.setObjectName(_fromUtf8("MainWidget"))
        MainWidget.resize(560, 428)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Fixed, QtGui.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(MainWidget.sizePolicy().hasHeightForWidth())
        MainWidget.setSizePolicy(sizePolicy)
        MainWidget.setMinimumSize(QtCore.QSize(170, 0))
        MainWidget.setMaximumSize(QtCore.QSize(16777215, 16777215))
        MainWidget.setStyleSheet(_fromUtf8("background-color: rgb(0, 0, 0);\n"
"color: rgb(255, 255, 255);\n"
"font: 10pt \"Droid Sans\";"))
        self.horizontalLayout_2 = QtGui.QHBoxLayout(MainWidget)
        self.horizontalLayout_2.setObjectName(_fromUtf8("horizontalLayout_2"))
        self.horizontalLayout_8 = QtGui.QHBoxLayout()
        self.horizontalLayout_8.setObjectName(_fromUtf8("horizontalLayout_8"))
        self.plotVertLayout = QtGui.QVBoxLayout()
        self.plotVertLayout.setObjectName(_fromUtf8("plotVertLayout"))
        self.vitalsPlot = PlotWidget(MainWidget)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.MinimumExpanding, QtGui.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.vitalsPlot.sizePolicy().hasHeightForWidth())
        self.vitalsPlot.setSizePolicy(sizePolicy)
        self.vitalsPlot.setMinimumSize(QtCore.QSize(150, 8))
        self.vitalsPlot.setStyleSheet(_fromUtf8("background-color: rgb(170, 0, 0);"))
        self.vitalsPlot.setFrameShape(QtGui.QFrame.NoFrame)
        self.vitalsPlot.setFrameShadow(QtGui.QFrame.Plain)
        self.vitalsPlot.setLineWidth(0)
        self.vitalsPlot.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.vitalsPlot.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.vitalsPlot.setObjectName(_fromUtf8("vitalsPlot"))
        self.plotVertLayout.addWidget(self.vitalsPlot)
        self.slowPlotHorizLayout = QtGui.QHBoxLayout()
        self.slowPlotHorizLayout.setObjectName(_fromUtf8("slowPlotHorizLayout"))
        self.textBrowser = QtGui.QTextBrowser(MainWidget)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Fixed, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.textBrowser.sizePolicy().hasHeightForWidth())
        self.textBrowser.setSizePolicy(sizePolicy)
        self.textBrowser.setMinimumSize(QtCore.QSize(136, 30))
        self.textBrowser.setMaximumSize(QtCore.QSize(136, 30))
        self.textBrowser.setFrameShape(QtGui.QFrame.NoFrame)
        self.textBrowser.setFrameShadow(QtGui.QFrame.Plain)
        self.textBrowser.setLineWidth(0)
        self.textBrowser.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.textBrowser.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.textBrowser.setObjectName(_fromUtf8("textBrowser"))
        self.slowPlotHorizLayout.addWidget(self.textBrowser)
        self.slowUpdatePeriod = QtGui.QDoubleSpinBox(MainWidget)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Fixed, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.slowUpdatePeriod.sizePolicy().hasHeightForWidth())
        self.slowUpdatePeriod.setSizePolicy(sizePolicy)
        self.slowUpdatePeriod.setMinimumSize(QtCore.QSize(150, 0))
        self.slowUpdatePeriod.setMaximumSize(QtCore.QSize(150, 16777215))
        self.slowUpdatePeriod.setStyleSheet(_fromUtf8("background-color: rgb(179, 178, 178);\n"
"border-color: rgb(0, 0, 0);\n"
"alternate-background-color: rgb(179, 178, 178);"))
        self.slowUpdatePeriod.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.slowUpdatePeriod.setDecimals(1)
        self.slowUpdatePeriod.setMinimum(0.1)
        self.slowUpdatePeriod.setMaximum(1000000000.0)
        self.slowUpdatePeriod.setProperty("value", 1.0)
        self.slowUpdatePeriod.setObjectName(_fromUtf8("slowUpdatePeriod"))
        self.slowPlotHorizLayout.addWidget(self.slowUpdatePeriod)
        spacerItem = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.slowPlotHorizLayout.addItem(spacerItem)
        self.plotVertLayout.addLayout(self.slowPlotHorizLayout)
        self.pressurePlot = PlotWidget(MainWidget)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pressurePlot.sizePolicy().hasHeightForWidth())
        self.pressurePlot.setSizePolicy(sizePolicy)
        self.pressurePlot.setMinimumSize(QtCore.QSize(150, 10))
        self.pressurePlot.setStyleSheet(_fromUtf8("background-color: rgb(255, 0, 255);"))
        self.pressurePlot.setFrameShape(QtGui.QFrame.NoFrame)
        self.pressurePlot.setFrameShadow(QtGui.QFrame.Plain)
        self.pressurePlot.setLineWidth(0)
        self.pressurePlot.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.pressurePlot.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.pressurePlot.setObjectName(_fromUtf8("pressurePlot"))
        self.plotVertLayout.addWidget(self.pressurePlot)
        self.fastPlotHorizLayout = QtGui.QHBoxLayout()
        self.fastPlotHorizLayout.setObjectName(_fromUtf8("fastPlotHorizLayout"))
        self.textEdit = QtGui.QTextEdit(MainWidget)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Fixed, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.textEdit.sizePolicy().hasHeightForWidth())
        self.textEdit.setSizePolicy(sizePolicy)
        self.textEdit.setMinimumSize(QtCore.QSize(136, 30))
        self.textEdit.setMaximumSize(QtCore.QSize(136, 30))
        self.textEdit.setLayoutDirection(QtCore.Qt.RightToLeft)
        self.textEdit.setFrameShape(QtGui.QFrame.NoFrame)
        self.textEdit.setFrameShadow(QtGui.QFrame.Plain)
        self.textEdit.setLineWidth(0)
        self.textEdit.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.textEdit.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.textEdit.setObjectName(_fromUtf8("textEdit"))
        self.fastPlotHorizLayout.addWidget(self.textEdit)
        self.fastUpdatePeriod = QtGui.QDoubleSpinBox(MainWidget)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Fixed, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.fastUpdatePeriod.sizePolicy().hasHeightForWidth())
        self.fastUpdatePeriod.setSizePolicy(sizePolicy)
        self.fastUpdatePeriod.setMinimumSize(QtCore.QSize(100, 0))
        self.fastUpdatePeriod.setMaximumSize(QtCore.QSize(100, 16777215))
        self.fastUpdatePeriod.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.fastUpdatePeriod.setStyleSheet(_fromUtf8("background-color: rgb(179, 178, 178);"))
        self.fastUpdatePeriod.setFrame(True)
        self.fastUpdatePeriod.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.fastUpdatePeriod.setButtonSymbols(QtGui.QAbstractSpinBox.UpDownArrows)
        self.fastUpdatePeriod.setAccelerated(False)
        self.fastUpdatePeriod.setDecimals(1)
        self.fastUpdatePeriod.setMaximum(60.0)
        self.fastUpdatePeriod.setProperty("value", 5.0)
        self.fastUpdatePeriod.setObjectName(_fromUtf8("fastUpdatePeriod"))
        self.fastPlotHorizLayout.addWidget(self.fastUpdatePeriod)
        spacerItem1 = QtGui.QSpacerItem(170, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.fastPlotHorizLayout.addItem(spacerItem1)
        self.plotVertLayout.addLayout(self.fastPlotHorizLayout)
        self.horizontalLayout_8.addLayout(self.plotVertLayout)
        self.testVertLayout = QtGui.QVBoxLayout()
        self.testVertLayout.setSpacing(0)
        self.testVertLayout.setObjectName(_fromUtf8("testVertLayout"))
        self.maxPressureText = QtGui.QPlainTextEdit(MainWidget)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Fixed, QtGui.QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.maxPressureText.sizePolicy().hasHeightForWidth())
        self.maxPressureText.setSizePolicy(sizePolicy)
        self.maxPressureText.setMinimumSize(QtCore.QSize(170, 0))
        self.maxPressureText.setMaximumSize(QtCore.QSize(170, 25))
        self.maxPressureText.setStyleSheet(_fromUtf8("color: rgb(85, 255, 0);\n"
"font: 75 12pt \"Droid Sans\";"))
        self.maxPressureText.setFrameShape(QtGui.QFrame.NoFrame)
        self.maxPressureText.setFrameShadow(QtGui.QFrame.Plain)
        self.maxPressureText.setLineWidth(0)
        self.maxPressureText.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.maxPressureText.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.maxPressureText.setObjectName(_fromUtf8("maxPressureText"))
        self.testVertLayout.addWidget(self.maxPressureText)
        self.minPressureText = QtGui.QPlainTextEdit(MainWidget)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Fixed, QtGui.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.minPressureText.sizePolicy().hasHeightForWidth())
        self.minPressureText.setSizePolicy(sizePolicy)
        self.minPressureText.setMinimumSize(QtCore.QSize(170, 0))
        self.minPressureText.setMaximumSize(QtCore.QSize(170, 25))
        font = QtGui.QFont()
        font.setFamily(_fromUtf8("Droid Sans"))
        font.setPointSize(12)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(9)
        self.minPressureText.setFont(font)
        self.minPressureText.setStyleSheet(_fromUtf8("font: 75 12pt \"Droid Sans\";\n"
"color: rgb(255, 255, 0);"))
        self.minPressureText.setFrameShape(QtGui.QFrame.NoFrame)
        self.minPressureText.setFrameShadow(QtGui.QFrame.Plain)
        self.minPressureText.setLineWidth(0)
        self.minPressureText.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.minPressureText.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.minPressureText.setObjectName(_fromUtf8("minPressureText"))
        self.testVertLayout.addWidget(self.minPressureText)
        spacerItem2 = QtGui.QSpacerItem(20, 40, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.testVertLayout.addItem(spacerItem2)
        self.horizontalLayout_8.addLayout(self.testVertLayout)
        self.horizontalLayout_2.addLayout(self.horizontalLayout_8)

        self.retranslateUi(MainWidget)
        QtCore.QMetaObject.connectSlotsByName(MainWidget)

    def retranslateUi(self, MainWidget):
        MainWidget.setWindowTitle(_translate("MainWidget", "Animal Monitor v1.0", None))
        self.textBrowser.setHtml(_translate("MainWidget", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Droid Sans\'; font-size:10pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"right\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">Sampling Frequency:</p></body></html>", None))
        self.slowUpdatePeriod.setPrefix(_translate("MainWidget", "1 sample / ", None))
        self.slowUpdatePeriod.setSuffix(_translate("MainWidget", " sec", None))
        self.textEdit.setHtml(_translate("MainWidget", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Droid Sans\'; font-size:10pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"right\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">Plot Refresh Period:</p></body></html>", None))
        self.fastUpdatePeriod.setSuffix(_translate("MainWidget", " sec", None))
        self.maxPressureText.setPlainText(_translate("MainWidget", "Max Pres:\n"
"", None))
        self.minPressureText.setPlainText(_translate("MainWidget", "Min Pres:", None))

from pyqtgraph import PlotWidget
