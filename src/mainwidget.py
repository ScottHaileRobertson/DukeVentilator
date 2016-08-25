# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file '../QtCreatorUI/mainwidget.ui'
#
# Created: Wed Aug 24 13:11:23 2016
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
        MainWidget.resize(807, 488)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Fixed, QtGui.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(MainWidget.sizePolicy().hasHeightForWidth())
        MainWidget.setSizePolicy(sizePolicy)
        MainWidget.setMinimumSize(QtCore.QSize(170, 0))
        MainWidget.setMaximumSize(QtCore.QSize(16777215, 16777215))
        MainWidget.setStyleSheet(_fromUtf8("background-color: rgb(0, 0, 0);\n"
"color: rgb(255, 255, 255);\n"
"font: 10pt \"Droid Sans Mono\";"))
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
        self.fastUpdatePeriod.setMinimum(0.5)
        self.fastUpdatePeriod.setMaximum(20.0)
        self.fastUpdatePeriod.setProperty("value", 4.0)
        self.fastUpdatePeriod.setObjectName(_fromUtf8("fastUpdatePeriod"))
        self.fastPlotHorizLayout.addWidget(self.fastUpdatePeriod)
        spacerItem1 = QtGui.QSpacerItem(170, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.fastPlotHorizLayout.addItem(spacerItem1)
        self.plotVertLayout.addLayout(self.fastPlotHorizLayout)
        self.horizontalLayout_8.addLayout(self.plotVertLayout)
        self.testVertLayout = QtGui.QVBoxLayout()
        self.testVertLayout.setSpacing(0)
        self.testVertLayout.setObjectName(_fromUtf8("testVertLayout"))
        self.modeText = QtGui.QPlainTextEdit(MainWidget)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Fixed, QtGui.QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.modeText.sizePolicy().hasHeightForWidth())
        self.modeText.setSizePolicy(sizePolicy)
        self.modeText.setMinimumSize(QtCore.QSize(250, 0))
        self.modeText.setMaximumSize(QtCore.QSize(250, 60))
        font = QtGui.QFont()
        font.setFamily(_fromUtf8("Droid Sans Mono"))
        font.setPointSize(13)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.modeText.setFont(font)
        self.modeText.setStyleSheet(_fromUtf8("font: 13pt \"Droid Sans Mono\";"))
        self.modeText.setFrameShape(QtGui.QFrame.NoFrame)
        self.modeText.setFrameShadow(QtGui.QFrame.Plain)
        self.modeText.setLineWidth(0)
        self.modeText.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.modeText.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.modeText.setObjectName(_fromUtf8("modeText"))
        self.testVertLayout.addWidget(self.modeText)
        self.canulaText = QtGui.QPlainTextEdit(MainWidget)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Fixed, QtGui.QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.canulaText.sizePolicy().hasHeightForWidth())
        self.canulaText.setSizePolicy(sizePolicy)
        self.canulaText.setMinimumSize(QtCore.QSize(250, 0))
        self.canulaText.setMaximumSize(QtCore.QSize(250, 125))
        font = QtGui.QFont()
        font.setFamily(_fromUtf8("Droid Sans Mono"))
        font.setPointSize(13)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.canulaText.setFont(font)
        self.canulaText.setStyleSheet(_fromUtf8("font: 13pt \"Droid Sans Mono\";"))
        self.canulaText.setFrameShape(QtGui.QFrame.NoFrame)
        self.canulaText.setFrameShadow(QtGui.QFrame.Plain)
        self.canulaText.setLineWidth(0)
        self.canulaText.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.canulaText.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.canulaText.setObjectName(_fromUtf8("canulaText"))
        self.testVertLayout.addWidget(self.canulaText)
        self.nitrogenText = QtGui.QPlainTextEdit(MainWidget)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Fixed, QtGui.QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.nitrogenText.sizePolicy().hasHeightForWidth())
        self.nitrogenText.setSizePolicy(sizePolicy)
        self.nitrogenText.setMinimumSize(QtCore.QSize(250, 0))
        self.nitrogenText.setMaximumSize(QtCore.QSize(250, 60))
        self.nitrogenText.setStyleSheet(_fromUtf8("color: rgb(255, 255, 0);\n"
"font: 13pt \"Droid Sans Mono\";"))
        self.nitrogenText.setFrameShape(QtGui.QFrame.NoFrame)
        self.nitrogenText.setFrameShadow(QtGui.QFrame.Plain)
        self.nitrogenText.setLineWidth(0)
        self.nitrogenText.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.nitrogenText.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.nitrogenText.setObjectName(_fromUtf8("nitrogenText"))
        self.testVertLayout.addWidget(self.nitrogenText)
        self.oxygenText = QtGui.QPlainTextEdit(MainWidget)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Fixed, QtGui.QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.oxygenText.sizePolicy().hasHeightForWidth())
        self.oxygenText.setSizePolicy(sizePolicy)
        self.oxygenText.setMinimumSize(QtCore.QSize(250, 0))
        self.oxygenText.setMaximumSize(QtCore.QSize(250, 60))
        font = QtGui.QFont()
        font.setFamily(_fromUtf8("Droid Sans Mono"))
        font.setPointSize(13)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.oxygenText.setFont(font)
        self.oxygenText.setStyleSheet(_fromUtf8("color: rgb(85, 255, 0);\n"
"font: 13pt \"Droid Sans Mono\";"))
        self.oxygenText.setFrameShape(QtGui.QFrame.NoFrame)
        self.oxygenText.setFrameShadow(QtGui.QFrame.Plain)
        self.oxygenText.setLineWidth(0)
        self.oxygenText.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.oxygenText.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.oxygenText.setTabStopWidth(73)
        self.oxygenText.setObjectName(_fromUtf8("oxygenText"))
        self.testVertLayout.addWidget(self.oxygenText)
        self.hpText = QtGui.QPlainTextEdit(MainWidget)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Fixed, QtGui.QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.hpText.sizePolicy().hasHeightForWidth())
        self.hpText.setSizePolicy(sizePolicy)
        self.hpText.setMinimumSize(QtCore.QSize(250, 0))
        self.hpText.setMaximumSize(QtCore.QSize(250, 85))
        self.hpText.setStyleSheet(_fromUtf8("color: rgb(255, 85, 0);\n"
"font: 13pt \"Droid Sans Mono\";"))
        self.hpText.setFrameShape(QtGui.QFrame.NoFrame)
        self.hpText.setFrameShadow(QtGui.QFrame.Plain)
        self.hpText.setLineWidth(0)
        self.hpText.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.hpText.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.hpText.setObjectName(_fromUtf8("hpText"))
        self.testVertLayout.addWidget(self.hpText)
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
"</style></head><body style=\" font-family:\'Droid Sans Mono\'; font-size:10pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"right\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:\'Droid Sans\';\">Sampling Frequency:</span></p></body></html>", None))
        self.slowUpdatePeriod.setPrefix(_translate("MainWidget", "1 sample / ", None))
        self.slowUpdatePeriod.setSuffix(_translate("MainWidget", " sec", None))
        self.textEdit.setHtml(_translate("MainWidget", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Droid Sans Mono\'; font-size:10pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"right\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:\'Droid Sans\';\">Plot Refresh Period:</span></p></body></html>", None))
        self.fastUpdatePeriod.setSuffix(_translate("MainWidget", " sec", None))
        self.modeText.setPlainText(_translate("MainWidget", "Mode: ", None))
        self.canulaText.setPlainText(_translate("MainWidget", "Canula\n"
"Pmax: 0.0 cmH20\n"
"Pmin: 0.0 cmH20\n"
"TV  : 0.00 mL", None))
        self.nitrogenText.setPlainText(_translate("MainWidget", "Nitrogen\n"
"P: 0.0 psi  V: 0.01 mL", None))
        self.oxygenText.setPlainText(_translate("MainWidget", "Oxygen\n"
"P: 0.0 psi  V: 0.01 mL", None))
        self.hpText.setPlainText(_translate("MainWidget", "HP Gas\n"
"P: 0.0 psi  V: 0.01 mL", None))

from pyqtgraph import PlotWidget
