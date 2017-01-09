#!/usr/bin/python
from pyqtgraph.Qt import QtGui, QtCore
import numpy as np
import pyqtgraph as pg
from mainwidget import Ui_MainWidget
import datafetcher as df
import multiprocessing as mp
import time
import csv
import RPi.GPIO as GPIO

# Version
VERSION_STRING = "2.0"

class DataMonitoringWindow(QtGui.QWidget):
    def __init__(self):
        # Setup UI 
        QtGui.QWidget.__init__(self)
        self.ui = Ui_MainWidget()
        self.ui.setupUi(self)
        self.setWindowTitle("Duke Animal Monitor v" + VERSION_STRING)

        # CONSTANTS
        self.DATA_FETCH_PERIOD = 0.0005
        self.PLOT_UPDATE_PERIOD = 0.25  # TODO remove this... plot as fast as we can
        self.PLOT_TIME_RANGE = 6  # Total seconds of display #TODO make this the max length
        self.SLOW_PLOT_LINE_LENGTH = 250
        self.SLOW_PLOT_LINE_IDX = 0
        self.FAST_PLOT_LINE_LENGTH = 250
        self.FAST_PLOT_NLINES = math.ceil(self.PLOT_TIME_RANGE / self.DATA_FETCH_PERIOD) + 1
        self.FAST_PLOT_LINE_IDX = 0
        self.FAST_PLOT_POSITION_IDX = 0

        self.UPDATE_SLOW_BOUNDS_TIME = 20 # Sec (initially at least)
        self.UPDATE_SLOW_BOUNDS_PCT = 1.5
        self.slowBounds_minX = 0
        self.slowBounds_maxX = self.UPDATE_SLOW_BOUNDS_TIME

        self.TIME_SHIFT_PCT = 0.75
        self.TEXT_UPDATE_PERIOD = 1
        self.NEXT_TEXT_UPDATE = 1
        self.SLOW_UPDATE_PERIOD = 5
        self.NEXT_SLOW_UPDATE = 1

        self.MAX_DISPLAYED_TRIGGERS = 20

        self.start_time = 0
        self.writeThisHeartRate = ''
        self.writeThisAnimalTemp = ''

        self.csvfile = open('DukeVentilatorData_' + datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S-%f") + '.csv', 'w')
        self.csvwriter = csv.writer(self.csvfile)

        # Initialize array to store time stamps of heartbeats
        self.MAX_DISPLAYED_HEARTBEATS = 600 / 60 / self.PLOT_TIME_RANGE  # 600 BPM for max time
        self.N_HEARTBEATS_TO_AVG = 10
        self.MAX_TIME_BETWEEN_HEARTBEATS = 2
        self.HEART_BEATS_COUNTED = 0
        self.heartBeatArray = [0] * self.N_HEARTBEATS_TO_AVG
        self.heartBeatIdx = 0

        # Read in bore temperature data
        temp_sensor_cal = np.genfromtxt('temperature_calibration.csv', delimiter=',')
        temp_sensor_cal_raw = temp_sensor_cal[:, 0]
        temp_sensor_cal_temp = temp_sensor_cal[:, 1]
        self.tempSensor_fit = np.polyfit(temp_sensor_cal_temp, temp_sensor_cal_raw, 3)

        # Read in pressure calibration data
        canula_p_cal = np.genfromtxt('canulaPressure_calibration.csv', delimiter=',')
        canula_p_cal_raw = canula_p_cal[:, 0]
        canula_p_cal_p = canula_p_cal[:, 1]
        canulaP_lin_fit = np.polyfit(canula_p_cal_raw, canula_p_cal_p, 1)
        self.canulaPressureSlope = canulaP_lin_fit[0]
        self.canulaPressureIntercept = canulaP_lin_fit[1]

        regulatorP_cal = np.genfromtxt('regulatorPressure_calibration.csv', delimiter=',')
        regulatorP_cal_raw = regulatorP_cal[:, 0]
        regulatorP_cal_p = regulatorP_cal[:, 1]
        regulatorP_lin_fit = np.polyfit(regulatorP_cal_raw, regulatorP_cal_p, 1)
        self.regulatorPressureSlope = regulatorP_lin_fit[0]
        self.regulatorPressureIntercept = regulatorP_lin_fit[1]

        # Read in volume calibration data
        oxygenV_cal = np.genfromtxt('oxygenVolume_calibration.csv', delimiter=',')
        oxygenV_cal_raw = oxygenV_cal[:, 0]
        oxygenV_cal_p = oxygenV_cal[:, 1]
        oxygenV_lin_fit = np.polyfit(oxygenV_cal_raw, oxygenV_cal_p, 1)
        self.oxygenVolumeSlope = oxygenV_lin_fit[0]
        self.oxygenVolumeIntercept = oxygenV_lin_fit[1]

        nitrogenV_cal = np.genfromtxt('nitrogenVolume_calibration.csv', delimiter=',')
        nitrogenV_cal_raw = nitrogenV_cal[:, 0]
        nitrogenV_cal_p = nitrogenV_cal[:, 1]
        nitrogenV_lin_fit = np.polyfit(nitrogenV_cal_raw, nitrogenV_cal_p, 1)
        self.nitrogenVolumeSlope = nitrogenV_lin_fit[0]
        self.nitrogenVolumeIntercept = nitrogenV_lin_fit[1]

        hpGasV_cal = np.genfromtxt('hpGasVolume_calibration.csv', delimiter=',')
        hpGasV_cal_raw = hpGasV_cal[:, 0]
        hpGasV_cal_p = hpGasV_cal[:, 1]
        hpGasV_lin_fit = np.polyfit(hpGasV_cal_raw, hpGasV_cal_p, 1)
        self.hpGasVolumeSlope = hpGasV_lin_fit[0]
        self.hpGasVolumeIntercept = hpGasV_lin_fit[1]

        # Create data fetching process
        self.dataFetcher = df.TimedDataFetcher(self.DATA_FETCH_PERIOD)

        # Initialize graphs
        self.ventilationPen = pg.mkPen({'color': "0FF"})
        self.ui.ventilationPlot.setMouseEnabled(x=False, y=True)
        self.ui.ventilationPlot.enableAutoRange(x=False, y=True)
        self.ui.ventilationPlot.setLabel('left', "Canula Pressure", units='cmH20')
        self.ui.ventilationPlot.getAxis('left').enableAutoSIPrefix(False)
        self.ui.ventilationPlot.setLabel('bottom', "Time", units='sec')
        self.ui.ventilationPlot.setXRange(0, self.PLOT_TIME_RANGE, padding=0)
        self.ventilationLines = [PlotCurveItem(x=[], y=[], pen=self.ventilationPen, antialias=True) for i in
                                 range(self.FAST_PLOT_NLINES)]
        for i in range(self.FAST_PLOT_NLINES):
            self.ui.ventilationPlot.addItem(self.ventilationLines[i])

        # Initialize a bunch of lightweight trigger lines
        self.triggerPen = pg.mkPen({'color': "F0F"})
        self.triggerLines = [pg.InfiniteLine(pos=-1, angle=90, movable=False, pen=self.triggerPen, bounds=None) for i in
                             range(self.MAX_TRIGGERS_DISP)]
        self.triggerIdx = -1
        for i in range(self.MAX_TRIGGERS_DISP):
            self.ui.ventilationPlot.addItem(self.triggerLines[i], ignoreBounds=True)

        self.ecgPen = pg.mkPen({'color': "0FF"})
        self.ui.ecgPlot.setMouseEnabled(x=False, y=True)
        self.ui.ecgPlot.enableAutoRange(x=False, y=True)
        self.ui.ecgPlot.setLabel('left', "ECG")
        self.ui.ecgPlot.getAxis('left').enableAutoSIPrefix(False)
        self.ui.ecgPlot.setLabel('bottom', "Time", units='sec')
        self.ui.ecgPlot.setXRange(0, self.PLOT_TIME_RANGE, padding=0)
        self.ecgLines = [PlotCurveItem(x=[], y=[], pen=self.ecgPen, antialias=True) for i in
                         range(self.FAST_PLOT_NLINES)]
        for i in range(self.FAST_PLOT_NLINES):
            self.ui.ecgPlot.addItem(self.ecgLines[i])

        self.accum_time_vec = []
        self.accum_vent_vec = []
        self.accum_ecg_vec = []

        # Initialize a bunch of lightweight heartbeat lines
        self.heartBeatPen = pg.mkPen({'color': "C00"})
        self.heartBeatLines = [pg.InfiniteLine(pos=-1, angle=90, movable=False, pen=self.heartBeatPen, bounds=None) for
                               i in range(self.MAX_HEARTBEAT_DISP)]
        self.heartBeatLine_Idx = -1
        for i in range(self.MAX_HEARTBEAT_DISP):
            self.ui.ecgPlot.addItem(self.heartBeatLines[i], ignoreBounds=True)

        self.minXlim = 0
        self.updatePlotTimeRange()
        self.updateSlowPlotRefreshRate()
        self.maxXlim = self.minXlim + self.PLOT_TIME_RANGE

        # Initialize Min and Max pressure axis
        self.ui.vitalsPlot.setMouseEnabled(x=False, y=True)
        self.ui.vitalsPlot.enableAutoRange(x=False, y=True)
        self.pressureSlowPlot = self.ui.vitalsPlot.plotItem
        self.pressureSlowPlot.getAxis('left').setLabel("Canula Pressure", units='cmH20')
        self.pressureSlowPlot.getAxis('left').enableAutoSIPrefix(False)

        # Add empty lines
        self.pressureLinePen = pg.mkPen({'color': "FFF"})  # TODO consolidate all pens
        self.minPressureLine = pg.PlotCurveItem(x=[], y=[], pen=self.pressureLinePen, antialias=True)
        self.maxPressureLine = pg.PlotCurveItem(x=[], y=[], pen=self.pressureLinePen, antialias=True)
        self.pressureSlowPlot.addItem(self.minPressureLine)
        self.pressureSlowPlot.addItem(self.maxPressureLine)

        # Add heart rate axis
        self.heartRatePen = pg.mkPen({'color': "C00"}, width=2)
        self.pressureSlowPlot.showAxis('right')
        self.pressureSlowPlot.getAxis('right').setLabel("Heart Rate", units='BPM')
        self.pressureSlowPlot.getAxis('right').setPen(self.heartRatePen)
        self.heartrateSlowPlot = pg.ViewBox()
        self.pressureSlowPlot.scene().addItem(self.heartrateSlowPlot)
        self.pressureSlowPlot.getAxis('right').linkToView(self.heartrateSlowPlot)
        self.heartrateSlowPlot.setXLink(self.pressureSlowPlot)

        # Add temperature axis
        self.temperatureAxis = pg.AxisItem('right')
        self.temperatureAxis.setLabel('Temperature', 'C')
        self.temperaturePen = pg.mkPen({'color': "0C0"}, width=2)
        self.temperatureAxis.setPen(self.temperaturePen)
        self.temperatureViewbox = pg.ViewBox()
        self.pressureSlowPlot.layout.addItem(self.temperatureAxis, 2, 3)
        self.pressureSlowPlot.scene().addItem(self.temperatureViewbox)
        self.temperatureAxis.linkToView(self.temperatureViewbox)
        self.temperatureViewbox.setXLink(self.pressureSlowPlot)

        self.updateViews()
        self.pressureSlowPlot.vb.sigResized.connect(self.updateViews)

        # Setup dynamic plotting process
        # TODO make this plot as fast as possible
        self.isGraphing = False
        self.timer = pg.QtCore.QTimer()
        self.timer.timeout.connect(self.updateUI)
        self.timer.start(self.PLOT_UPDATE_PERIOD)

        # Listen to events from plot time range drop down menus
        self.ui.fastUpdatePeriod.valueChanged.connect(self.updatePlotTimeRange)
        self.ui.slowUpdatePeriod.valueChanged.connect(self.updateSlowPlotRefreshRate)

        # Set GPIO mode to board so things go smoothly with RPi upgrades
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(5, GPIO.IN)
        GPIO.add_event_detect(5, GPIO.BOTH, callback=self.hpVsO2Changed, bouncetime=500)
        GPIO.setup(13, GPIO.IN)
        GPIO.add_event_detect(13, GPIO.BOTH, callback=self.triggerChanged, bouncetime=500)
        GPIO.setup(26, GPIO.IN)
        GPIO.add_event_detect(26, GPIO.BOTH, callback=self.heartBeatDetected, bouncetime=500)
        self.nitrogenModeOn = GPIO.input(5)

    def updateViews(self):
        self.heartrateSlowPlot.setGeometry(self.pressureSlowPlot.getViewBox().sceneBoundingRect())
        self.heartrateSlowPlot.linkedViewChanged(self.pressureSlowPlot.getViewBox(), self.heartrateSlowPlot.XAxis)
        self.temperatureViewbox.setGeometry(self.pressureSlowPlot.getViewBox().sceneBoundingRect())
        self.temperatureViewbox.linkedViewChanged(self.pressureSlowPlot.getViewBox(), self.temperatureViewbox.XAxis)

    def heartBeatDetected(self, chan):
        trig_time = time.time() - self.start_time
        self.HEART_BEATS_COUNTED = self.HEART_BEATS_COUNTED + 1
        self.heartBeatArray[self.heartBeatIdx] = trig_time
        if (self.heartBeatIdx == (self.HEART_BEATS_TO_AVG - 1)):
            self.heartBeatIdx = 0  # Reset
        else:
            self.heartBeatIdx = self.heartBeatIdx + 1
        # Update heartBeat line
        self.heartBeatLine_Idx += 1
        while (self.heartBeatLine_Idx >= self.MAX_HEARTBEAT_DISP):
            self.heartBeatLine_Idx -= self.MAX_HEARTBEAT_DISP
        self.heartBeatLines[self.heartBeatLine_Idx].setValue(trig_time)

    def triggerChanged(self, chan):
        # Update next trigger line
        trig_time = time.time() - self.start_time
        self.triggerIdx += 1
        while (self.triggerIdx >= self.MAX_TRIGGERS_DISP):
            self.triggerIdx -= self.MAX_TRIGGERS_DISP
        self.triggerLines[self.triggerIdx].setValue(trig_time)

    def hpVsO2Changed(self, chan):
        time.sleep(0.5)
        self.nitrogenModeOn = GPIO.input(5)

    def updateSlowPlotRefreshRate(self):
        self.NEXT_SLOW_UPDATE = self.NEXT_SLOW_UPDATE - self.SLOW_UPDATE_PERIOD + self.ui.slowUpdatePeriod.value()
        self.SLOW_UPDATE_PERIOD = self.ui.slowUpdatePeriod.value()

    def updatePlotTimeRange(self):
        self.PLOT_TIME_RANGE = self.ui.fastUpdatePeriod.value()

        # Update axes
        self.maxXlim = self.minXlim + self.PLOT_TIME_RANGE
        self.ui.ventilationPlot.setXRange(self.minXlim, self.maxXlim, padding=0)

    def startGraphing(self):
        if (not self.isGraphing):
            self.isGraphing = True
            self.start_time = time.time()
            self.dataFetcher.newStartTime(self.start_time)
            self.dataFetcher.startFetching()
            self.timer.start(5)
        self.isGraphing = False

    def stopGraphing(self):
        if self.isGraphing:
            self.dataFetcher.stopFetching()
            self.isGraphing = False
            self.startTime = []
            self.timer.stop()

    def updateUI(self):
        elapsed_time = time.time() - self.start_time

        # Under lock, find how much data we have to read
        self.dataFetcher.indexLock.acquire()
        nDataToRead = np.int32(self.dataFetcher.sync_bufferLength.value)
        lastIdx = self.dataFetcher.sync_bufferEndIdx.value
        endRead = lastIdx + 1
        if (self.dataFetcher.sync_bufferFull.value):
            startRead = 0
        else:
            startRead = self.dataFetcher.sync_bufferStartIdx.value
        stopReading = min(startRead + nDataToRead, self.dataFetcher.BUFFERSIZE)
        self.dataFetcher.indexLock.release()


        # We can now copy from the buffer safely.
        temp_time_queue = [0]*nDataToRead
        temp_canula_queue = [0]*nDataToRead
        temp_ecg_queue = [0]*nDataToRead
        nDataToWriteToCsv = nDataToRead

        if (nDataToRead > 0):
            temp_time_queue[0:(stopReading - startRead - 1)] = self.dataFetcher.sync_time_buf[
                                                               startRead:(stopReading - 1)]
            temp_canula_queue[
            0:(stopReading - startRead - 1)] = self.canulaPressureSlope * self.dataFetcher.sync_canula_buf[startRead:(
            stopReading - 1)] + self.canulaPressureIntercept
            temp_ecg_queue[0:(stopReading - startRead - 1)] = self.dataFetcher.sync_ecg_buf[startRead:(stopReading - 1)]

            # In case data has wrapped... read the wrapped data too
            if (endRead <= startRead):
                temp_time_queue[stopReading:(stopReading + endRead - 1)] = self.dataFetcher.sync_time_buf[
                                                                           0:(endRead - 1)]
                temp_canula_queue[
                stopReading:(stopReading + endRead - 1)] = self.canulaPressureSlope * self.dataFetcher.sync_canula_buf[
                                                                                      0:(
                                                                                      endRead - 1)] + self.canulaPressureIntercept
                temp_ecg_queue[stopReading:(stopReading + endRead - 1)] = self.dataFetcher.sync_ecg_buf[0:(endRead - 1)]

            # Data is copied, so move pointer to free up buffer space under lock
            self.dataFetcher.indexLock.acquire()
            self.dataFetcher.sync_bufferStartIdx.value = endRead
            self.dataFetcher.sync_bufferLength.value = self.dataFetcher.sync_bufferLength.value - nDataToRead
            self.dataFetcher.indexLock.release()

            # Now we can take all the time we want to plot the data; the
            # fetching process will keep taking new data while this slow plot
            # operation occurs. Nevertheless, we try to update the plotted
            # line quickly
            startReadPosition = 0
            while (nDataToRead > 0):
                spaceLeftInLine = self.FAST_PLOT_LINE_LENGTH - self.FAST_PLOT_POSITION_IDX

                # TODO make sure this actually works
                self.accum_time_vec = []
                self.accum_vent_vec = []
                self.accum_ecg_vec = []

                while (nDataToRead >= spaceLeftInLine):
                    dataToAddToThisLine = min(nDataToRead, spaceLeftInLine)
                    self.accum_time_vec.append(temp_time_queue[startReadPosition:(startReadPosition + dataToAddToThisLine - 1)])
                    self.accum_vent_vec.append(temp_canula_queue[startReadPosition:(startReadPosition + dataToAddToThisLine - 1)])
                    self.accum_ecg_vec.append(temp_ecg_queue[startReadPosition:(startReadPosition + dataToAddToThisLine - 1)])

                    # Plot new data
                    self.ecgLines[self.FAST_PLOT_LINE_IDX].setData(self.accum_time_vec, self.accum_ecg_vec)
                    self.ventilationLines[self.FAST_PLOT_LINE_IDX].setData(self.accum_time_vec, self.accum_ecg_vec)

                    # Update to next line
                    self.accum_time_vec = self.accum_time_vec[self.FAST_PLOT_POSITION_IDX - 1]
                    self.accum_vent_vec = self.accum_vent_vec[self.FAST_PLOT_POSITION_IDX - 1]
                    self.accum_ecg_vec = self.accum_ecg_vec[self.FAST_PLOT_POSITION_IDX - 1]
                    self.FAST_PLOT_LINE_IDX = self.FAST_PLOT_LINE_IDX + 1
                    self.FAST_PLOT_POSITION_IDX = 1  # First index is taken up by previous line's last point to connect them
                    spaceLeftInLine = self.FAST_PLOT_LINE_LENGTH - self.FAST_PLOT_POSITION_IDX

                    # Update ammount of data to plot
                    nDataToRead = nDataToRead - dataToAddToThisLine

                # Add last little bit of data
                if (nDataToRead > 0):
                    self.accum_time_vec.append(temp_time_queue[startReadPosition:(startReadPosition + nDataToRead - 1)])
                    self.accum_vent_vec.append(temp_canula_queue[startReadPosition:(startReadPosition + nDataToRead - 1)])
                    self.accum_ecg_vec.append(temp_ecg_queue[startReadPosition:(startReadPosition + nDataToRead - 1)])

                    # Plot new data
                    self.ecgLines[self.FAST_PLOT_LINE_IDX].setData(self.accum_time_vec, self.accum_ecg_vec)
                    self.ventilationLines[self.FAST_PLOT_LINE_IDX].setData(self.accum_time_vec, self.accum_ecg_vec)

                    # Update position index
                    self.FAST_PLOT_POSITION_IDX = self.FAST_PLOT_POSITION_IDX + nDataToRead
                    nDataToRead = 0

        # Update time axis if necessary
        if (elapsed_time > self.maxXlim):
            # Update axes
            while (elapsed_time > self.maxXlim):
                self.minXlim += self.TIME_SHIFT_PCT * self.PLOT_TIME_RANGE
                self.maxXlim = self.minXlim + self.PLOT_TIME_RANGE
            if(self.ui.ventilationPlot.autoRangeEnabled()):
                self.ui.ventilationPlot.setXRange(self.minXlim, self.maxXlim, padding=0)
            if (self.ui.ecgPlot.autoRangeEnabled()):
                self.ui.ecgPlot.setXRange(self.minXlim, self.maxXlim, padding=0)

        if (elapsed_time > self.slowBounds_maxX):
            # Update axes
            while (elapsed_time > self.slowBounds_maxX):
                self.slowBounds_maxX = round(self.slowBounds_maxX * self.UPDATE_SLOW_BOUNDS_PCT)
            if (self.ui.vitalsPlot.autoRangeEnabled()):
                self.ui.vitalsPlot.setXRange(self.slowBounds_minX, self.slowBounds_maxX, padding=0)
                # self.pressureSlowPlot.setXRange(self.slowBounds_minX, self.slowBounds_maxX, padding=0)

        # Update text and slow graph
        if (elapsed_time > min(self.NEXT_TEXT_UPDATE, self.NEXT_SLOW_UPDATE)):
            min_val = min(self.canula_queue)
            max_val = max(self.canula_queue)

            # Calculate pressures
            oxygen_pressure = self.regulatorPressureSlope * self.dataFetcher.getDataFromChannel(
                1) + self.regulatorPressureIntercept
            nitrogen_pressure = self.regulatorPressureSlope * self.dataFetcher.getDataFromChannel(
                2) + self.regulatorPressureIntercept
            hpGas_pressure = self.regulatorPressureSlope * self.dataFetcher.getDataFromChannel(
                3) + self.regulatorPressureIntercept

            # Calculate volumes
            oxygen_volume = oxygen_pressure * self.oxygenVolumeSlope + self.oxygenVolumeIntercept
            nitrogen_volume = nitrogen_pressure * self.nitrogenVolumeSlope + self.nitrogenVolumeIntercept
            hpGas_volume = hpGas_pressure * self.hpGasVolumeSlope + self.hpGasVolumeIntercept

            tidal_vol = oxygen_volume
            if (self.nitrogenModeOn):
                tidal_vol += nitrogen_volume
                mode_string = "Mode: Nitrogen & Oxygen"
            else:
                tidal_vol += hpGas_volume
                mode_string = "Mode: HP Gas & Oxygen"

            if (elapsed_time > self.NEXT_SLOW_UPDATE):
                while (elapsed_time > self.NEXT_SLOW_UPDATE):
                    self.NEXT_SLOW_UPDATE += self.SLOW_UPDATE_PERIOD

            if (elapsed_time > self.NEXT_TEXT_UPDATE):
                while (elapsed_time > self.NEXT_TEXT_UPDATE):
                    self.NEXT_TEXT_UPDATE += self.TEXT_UPDATE_PERIOD
                self.ui.nitrogenText.setPlainText(
                    "Nitrogen\nP: %3.1f psi  V: %4.2f mL" % (nitrogen_pressure, nitrogen_volume))
                self.ui.oxygenText.setPlainText("Oxygen\nP: %3.1f psi  V: %4.2f mL" % (oxygen_pressure, oxygen_volume))
                self.ui.hpText.setPlainText("HP Gas\nP: %3.1f psi  V: %4.2f mL" % (hpGas_pressure, hpGas_volume))
                self.ui.canulaText.setPlainText(
                    "Canula\nPmax: %3.1f cmH20\nPmin: %3.1f cmH20\nTV  : %4.2f mL" % (max_val, min_val, tidal_vol))
                self.ui.modeText.setPlainText(mode_string)

                # Crude check that animal is alive
                timeSinceLastRecordedHeartBeat = (
                time.time() - self.start_time - self.heartBeatArray[self.heartBeatIdx - 1])
                if (timeSinceLastRecordedHeartBeat > self.MAX_TIME_BETWEEN_HEARTBEATS):
                    self.HEART_BEATS_COUNTED = 0
                    self.heartBeatArray = [0] * self.HEART_BEATS_TO_AVG
                    self.heartBeatIdx = 0  # Reset
                    self.writeThisHeartRate = 'N/A'
                if (self.HEART_BEATS_COUNTED == 0):
                    self.ui.heartRateText.setPlainText("Heart Rate: NOT DETECTED")
                    if (self.CURRENT_HEARTRATE_COUNT != 0):
                        self.CURRENT_HEARTRATE_DATA = []
                        self.CURRENT_HEARTRATE_TIME = []
                        self.CURRENT_HEARTRATE_COUNT = 0
                        self.CURRENT_HEARTRATE_LINE = pg.PlotCurveItem(x=[], y=[], pen=self.heartRatePen,
                                                                       antialias=True)
                        self.heartrateSlowPlot.addItem(self.CURRENT_HEARTRATE_LINE)
                    self.writeThisHeartRate = 'N/A'
                elif (self.HEART_BEATS_COUNTED < self.HEART_BEATS_TO_AVG):
                    self.ui.heartRateText.setPlainText("Heart Rate: Calculating")
                    if (self.CURRENT_HEARTRATE_COUNT != 0):
                        self.CURRENT_HEARTRATE_DATA = []
                        self.CURRENT_HEARTRATE_TIME = []
                        self.CURRENT_HEARTRATE_COUNT = 0
                        self.CURRENT_HEARTRATE_LINE = pg.PlotCurveItem(x=[], y=[], pen=self.heartRatePen,
                                                                       antialias=True)
                        self.heartrateSlowPlot.addItem(self.CURRENT_HEARTRATE_LINE)
                    self.writeThisHeartRate = 'N/A'
                else:
                    # Calculate and update heart rate
                    timeForAllBeats = (max(self.heartBeatArray) - min(self.heartBeatArray))
                    self.writeThisHeartRate = 'N/A'
                    if (timeForAllBeats > 0):
                        heartRate = 60 * (self.HEART_BEATS_TO_AVG - 1) / timeForAllBeats
                        self.writeThisHeartRate = heartRate
                        heart_rate_string = "Heart Rate: %4.1f BPM" % heartRate
                        self.ui.heartRateText.setPlainText(heart_rate_string)

                        self.CURRENT_HEARTRATE_DATA.append(heartRate)
                        self.CURRENT_HEARTRATE_TIME.append(elapsed_time)
                        self.CURRENT_HEARTRATE_COUNT = self.CURRENT_HEARTRATE_COUNT + 1
                        if (self.CURRENT_HEARTRATE_COUNT > 1):
                            self.CURRENT_HEARTRATE_LINE.setData(self.CURRENT_HEARTRATE_TIME,
                                                                self.CURRENT_HEARTRATE_DATA)
                        if (self.CURRENT_HEARTRATE_COUNT == (self.SLOW_PLOT_LINE_LENGTH - 1)):
                            self.CURRENT_HEARTRATE_DATA = [heartRate]
                            self.CURRENT_HEARTRATE_TIME = [elapsed_time]
                            self.CURRENT_HEARTRATE_COUNT = 1
                            self.CURRENT_HEARTRATE_LINE = pg.PlotCurveItem(x=[], y=[], pen=self.heartRatePen,
                                                                           antialias=True)
                            self.heartrateSlowPlot.addItem(self.CURRENT_HEARTRATE_LINE)

                # Calculate Temperatures
                bore_temp_celcius = np.polyval(self.tempSensor_fit, self.dataFetcher.getDataFromChannel(7))
                animal_temp_celcius = np.polyval(self.tempSensor_fit, self.dataFetcher.getDataFromChannel(6))
                tempTextString_celcius = u"Bore Temp: %4.1f (\u00B0C)\nAnimal Temp: %4.1f (\u00B0C)" % (
                bore_temp_celcius, animal_temp_celcius)
                # self.ui.temperatureText.setPlainText(tempTextString_celcius)

                bore_temp_fahrenheit = bore_temp_celcius * (9 / 5) + 32
                animal_temp_fahrenheit = animal_temp_celcius * (9 / 5) + 32
                tempTextString_fahrenheit = u"Bore Temp: %4.1f (\u00B0C)\nAnimal Temp: %4.1f (\u00B0C)" % (
                bore_temp_fahrenheit, animal_temp_fahrenheit)
                self.ui.temperatureText.setPlainText(tempTextString_fahrenheit)
                self.writeThisAnimalTemp = animal_temp_fahrenheit

                self.CURRENT_TIME_DATA.append(elapsed_time)
                self.CURRENT_MAX_PRESSURE_DATA.append(max_val)
                self.CURRENT_MIN_PRESSURE_DATA.append(min_val)
                self.CURRENT_TEMPERATURE_DATA.append(animal_temp_fahrenheit)

                self.CURRENT_MIN_PRESSURE_LINE.setData(self.CURRENT_TIME_DATA, self.CURRENT_MIN_PRESSURE_DATA)
                self.CURRENT_MAX_PRESSURE_LINE.setData(self.CURRENT_TIME_DATA, self.CURRENT_MAX_PRESSURE_DATA)
                self.CURRENT_TEMPERATURE_LINE.setData(self.CURRENT_TIME_DATA, self.CURRENT_TEMPERATURE_DATA)
                self.SLOW_PLOT_LINE_IDX = self.SLOW_PLOT_LINE_IDX + 1

                if (self.SLOW_PLOT_LINE_IDX == (self.SLOW_PLOT_LINE_LENGTH - 1)):
                    # Initialize next data with current values so lines conect
                    self.CURRENT_TIME_DATA = [elapsed_time]
                    self.CURRENT_MAX_PRESSURE_DATA = [max_val]
                    self.CURRENT_MIN_PRESSURE_DATA = [min_val]
                    self.CURRENT_TEMPERATURE_DATA = [animal_temp_fahrenheit]

                    self.CURRENT_MIN_PRESSURE_LINE = pg.PlotCurveItem(x=[], y=[], pen=self.pressureLinePen,
                                                                      antialias=True)
                    self.CURRENT_MAX_PRESSURE_LINE = pg.PlotCurveItem(x=[], y=[], pen=self.pressureLinePen,
                                                                      antialias=True)
                    self.CURRENT_TEMPERATURE_LINE = pg.PlotCurveItem(x=[], y=[], pen=self.temperaturePen,
                                                                     antialias=True)
                    self.pressureSlowPlot.addItem(self.CURRENT_MIN_PRESSURE_LINE)
                    self.pressureSlowPlot.addItem(self.CURRENT_MAX_PRESSURE_LINE)
                    self.temperatureViewbox.addItem(self.CURRENT_TEMPERATURE_LINE)
                    self.SLOW_PLOT_LINE_IDX = 1  # RESET

        finishedPlotting_time = time.time() - self.start_time
        # Write to csvfile
        for i in range(nDataToWriteToCsv):
            self.csvwriter.writerow([temp_time_queue[i], temp_canula_queue[i], temp_ecg_queue[i], self.writeThisAnimalTemp ,self.writeThisHeartRate])

        finishedCSV_time = time.time() - self.start_time
        print "PLOTTED IN %4.8f seconds, wrote CSV file in %4.8f seconds" % ((finishedPlotting_time-elapsed_time),(finishedCSV_time -finishedPlotting_time))

    def closeEvent(self, ce):
        self.stopGraphing()
        self.dataFetcher.stopFetching()
        self.csvfile.close()


## Start Qt event loop unless running in interactive mode.
if __name__ == '__main__':
    import sys

    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        app = QtGui.QApplication([])
        win = DataMonitoringWindow()
        win.show()
        win.startGraphing()
        sys.exit(app.exec_())
