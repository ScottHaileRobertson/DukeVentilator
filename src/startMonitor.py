#!/usr/bin/python
from pyqtgraph.Qt import QtGui, QtCore
import numpy as np
import pyqtgraph as pg
from mainwidget import Ui_MainWidget
import datafetcher as df
import multiprocessing as mp 
import time
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
        self.MIN_DATA_FETCH_PERIOD = 0.0005
        self.MIN_PLOT_UPDATE_PERIOD = 0.25 # A little more than 60Hz
        self.PLOT_TIME_RANGE = 6 # Total seconds of display
        self.QUEUE_SIZE = int(round(self.PLOT_TIME_RANGE/self.MIN_DATA_FETCH_PERIOD))
        self.TIME_SHIFT_PCT = 0.75
        self.TEXT_UPDATE_PERIOD = 1
        self.NEXT_TEXT_UPDATE = 1
        self.SLOW_UPDATE_PERIOD = 5
        self.NEXT_SLOW_UPDATE = 1 
        self.MAX_TRIGGERS_DISP = 200
        self.MAX_HEARTBEAT_DISP = 200
        self.HEART_BEATS_TO_AVG = 50
        self.MAX_TIME_BETWEEN_HEARTBEATS = 30
        self.start_time = 0
        self.HEART_BEATS_COUNTED = 0
        
        # Initialize array to store time stamps of heartbeats
        self.heartBeatArray = [0]*self.HEART_BEATS_TO_AVG
        self.heartBeatIdx = 0
        
        # Read in bore temperature data
        tempSensor_cal = np.genfromtxt('temperature_calibration.csv', delimiter=',')
        tempSensor_cal_raw = tempSensor_cal[:,0]
        tempSensor_cal_temp = tempSensor_cal[:,1]
        self.tempSensor_fit = np.polyfit(tempSensor_cal_temp, tempSensor_cal_raw, 3)
        
        # Read in pressure calibration data
        canulaP_cal = np.genfromtxt('canulaPressure_calibration.csv', delimiter=',')
        canulaP_cal_raw = canulaP_cal[:,0]
        canulaP_cal_p = canulaP_cal[:,1]
        canulaP_lin_fit = np.polyfit(canulaP_cal_raw,canulaP_cal_p,1)
        self.canulaPressureSlope = canulaP_lin_fit[0]
        self.canulaPressureIntercept = canulaP_lin_fit[1]        
        #print "CANULA PRESSURE CALIBRATION: slope=%f intercept=%f" % (self.canulaPressureSlope, self.canulaPressureIntercept)

        regulatorP_cal = np.genfromtxt('regulatorPressure_calibration.csv', delimiter=',')
        regulatorP_cal_raw = regulatorP_cal[:,0]
        regulatorP_cal_p = regulatorP_cal[:,1]
        regulatorP_lin_fit = np.polyfit(regulatorP_cal_raw, regulatorP_cal_p,1)
        self.regulatorPressureSlope = regulatorP_lin_fit[0]
        self.regulatorPressureIntercept = regulatorP_lin_fit[1]
        #print "REGULATOR PRESSURE CALIBRATION: slope=%f intercept=%f" % (self.regulatorPressureSlope,self.regulatorPressureIntercept)
        
        # Read in volume calibration data
        oxygenV_cal = np.genfromtxt('oxygenVolume_calibration.csv', delimiter=',')
        oxygenV_cal_raw = oxygenV_cal[:,0]
        oxygenV_cal_p = oxygenV_cal[:,1]
        oxygenV_lin_fit = np.polyfit(oxygenV_cal_raw, oxygenV_cal_p,1)
        self.oxygenVolumeSlope = oxygenV_lin_fit[0]
        self.oxygenVolumeIntercept = oxygenV_lin_fit[1]
        #print "OXYGEN VOLUME CALIBRATION: slope=%f intercept=%f" % (self.oxygenVolumeSlope, self.oxygenVolumeIntercept)
        
        nitrogenV_cal = np.genfromtxt('nitrogenVolume_calibration.csv', delimiter=',')
        nitrogenV_cal_raw = nitrogenV_cal[:,0]
        nitrogenV_cal_p = nitrogenV_cal[:,1]
        nitrogenV_lin_fit = np.polyfit(nitrogenV_cal_raw, nitrogenV_cal_p,1)
        self.nitrogenVolumeSlope = nitrogenV_lin_fit[0]
        self.nitrogenVolumeIntercept = nitrogenV_lin_fit[1]
        #print "NITROGEN VOLUME CALIBRATION: slope=%f intercept=%f" % (self.nitrogenVolumeSlope, self.nitrogenVolumeIntercept)
        
        hpGasV_cal = np.genfromtxt('hpGasVolume_calibration.csv', delimiter=',')
        hpGasV_cal_raw = hpGasV_cal[:,0]
        hpGasV_cal_p = hpGasV_cal[:,1]
        hpGasV_lin_fit = np.polyfit(hpGasV_cal_raw, hpGasV_cal_p,1)
        self.hpGasVolumeSlope = hpGasV_lin_fit[0]
        self.hpGasVolumeIntercept = hpGasV_lin_fit[1]
        #print "HP GAS VOLUME CALIBRATION: slope=%f intercept=%f" % (self.hpGasVolumeSlope, self.hpGasVolumeIntercept)
        
        
        # Create data fetching process
        self.dataFetcher = df.TimedDataFetcher(self.MIN_DATA_FETCH_PERIOD)
                
        # Setup queues for plotting data as all NaNs
        self.time_queue = np.zeros(self.QUEUE_SIZE)
        self.canula_queue = np.zeros(self.QUEUE_SIZE)
        self.ecg_queue = np.zeros(self.QUEUE_SIZE)
        #self.temp_queue = np.zeros(self.QUEUE_SIZE)

        # Setup for slow plot data
        self.slow_time_queue = []
        self.slow_minPressure_queue = []
        self.slow_maxPressure_queue = []
        self.slow_tidalVolume_queue = []
        self.slow_heartRate_queue = []
        self.slow_timeHR_queue = []
        
        self.ui.fastUpdatePeriod.valueChanged.connect(self.updatePlotTimeRange)
        self.ui.slowUpdatePeriod.valueChanged.connect(self.updateSlowPlotRefreshRate)
        
        # Initialize graphs
        self.ui.pressurePlot.setMouseEnabled(x=False, y=True)
        self.ui.pressurePlot.enableAutoRange(x=False,y=True)
        self.ui.pressurePlot.setLabel('left', "Canula Pressure", units='cmH20')
        self.ui.pressurePlot.getAxis('left').enableAutoSIPrefix(False)
        self.ui.pressurePlot.setLabel('bottom', "Time", units='sec')
        self.ui.ecgPlot.setMouseEnabled(x=False, y=True)
        self.ui.ecgPlot.enableAutoRange(x=False,y=True)
        self.ui.ecgPlot.setLabel('left', "ECG")
        self.ui.ecgPlot.getAxis('left').enableAutoSIPrefix(False)
        self.ui.ecgPlot.setLabel('bottom', "Time", units='sec')
        self.minXlim = 0
        self.updatePlotTimeRange()
        self.updateSlowPlotRefreshRate()
        self.maxXlim = self.minXlim + self.PLOT_TIME_RANGE
        self.ui.pressurePlot.setXRange(0,self.PLOT_TIME_RANGE,padding=0)
        self.ui.ecgPlot.setXRange(0,self.PLOT_TIME_RANGE,padding=0)
        
        # Initialize pressure line
        self.pressureLine = pg.PlotCurveItem(x=[],y=[], \
           pen=pg.mkPen({'color': "0FF"}),antialias=True)
        self.ui.pressurePlot.addItem(self.pressureLine)
        
        # Initialize a bunch of lightweight trigger lines
        self.triggerPen = pen=pg.mkPen({'color': "F0F"})
        self.triggerLines = [ pg.InfiniteLine(pos=-1,angle=90,movable=False, \
           pen=self.triggerPen,bounds=None) for i in range(self.MAX_TRIGGERS_DISP)]
        self.triggerIdx = -1;
        for i in range(self.MAX_TRIGGERS_DISP):
          self.ui.pressurePlot.addItem(self.triggerLines[i],ignoreBounds=True)
        
        # Initialize ECG line
        self.ecgLine = pg.PlotCurveItem(x=[],y=[], \
           pen=pg.mkPen({'color': "0FF"}),antialias=True)
        self.ui.ecgPlot.addItem(self.ecgLine)
        
        # Initialize a bunch of lightweight heartbeat lines
        self.heartBeatPen = pen=pg.mkPen({'color': "C00"})
        self.heartBeatLines = [ pg.InfiniteLine(pos=-1,angle=90,movable=False, \
           pen=self.heartBeatPen,bounds=None) for i in range(self.MAX_HEARTBEAT_DISP)]
        self.heartBeatLine_Idx = -1
        for i in range(self.MAX_HEARTBEAT_DISP):
          self.ui.ecgPlot.addItem(self.heartBeatLines[i],ignoreBounds=True)
        
        # Initialize Min and Max pressure axis
        self.pressureSlowPlot = self.ui.vitalsPlot.plotItem
        self.pressureSlowPlot.getAxis('left').setLabel("Canula Pressure", units='cmH20')
        self.pressureSlowPlot.getAxis('left').enableAutoSIPrefix(False)

        # Add empty lines
        self.minPressureLine = pg.PlotCurveItem(x=[],y=[], \
           pen=pg.mkPen({'color': "FFF"}),antialias=True)
        self.maxPressureLine = pg.PlotCurveItem(x=[],y=[], \
           pen=pg.mkPen({'color': "FFF"}),antialias=True)
        self.pressureSlowPlot.addItem(self.minPressureLine)
        self.pressureSlowPlot.addItem(self.maxPressureLine)
                       
        # Add heart rate axis
        self.pressureSlowPlot.showAxis('right')
        self.pressureSlowPlot.getAxis('right').setLabel("Heart Rate", units='BPM')
        self.hearRatePen = pg.mkPen({'color': "C00"},width=2)
        self.pressureSlowPlot.getAxis('right').setPen(self.hearRatePen)
        
        self.heartrateSlowPlot = pg.ViewBox()
        self.pressureSlowPlot.scene().addItem(self.heartrateSlowPlot)
        self.pressureSlowPlot.getAxis('right').linkToView(self.heartrateSlowPlot)
        self.heartrateSlowPlot.setXLink(self.pressureSlowPlot)
                
        # Add empty heart rate line
        self.heartRateLine = pg.PlotCurveItem(x=[],y=[], \
           pen=self.hearRatePen,antialias=True)

        self.heartrateSlowPlot.addItem(self.heartRateLine)

        # Add temperature axis to right
        self.temperatureAxis = pg.AxisItem('right')
        self.temperatureAxis.setLabel('Temperature','C')
        self.temperaturePen = pg.mkPen({'color': "0C0"},width=2)
        self.temperatureAxis.setPen(self.temperaturePen)
        self.temperatureViewbox = pg.ViewBox()
        self.pressureSlowPlot.layout.addItem(self.temperatureAxis,2,3)
        self.pressureSlowPlot.scene().addItem(self.temperatureViewbox)
        self.temperatureAxis.linkToView(self.temperatureViewbox)
        self.temperatureViewbox.setXLink(self.pressureSlowPlot)

        # Add empty heart rate line
        self.temperatureLine = pg.PlotCurveItem(x=[1,5],y=[0,5], \
           pen=self.temperaturePen,antialias=True)
        self.temperatureViewbox.addItem(self.temperatureLine)

        self.updateViews();
        self.pressureSlowPlot.vb.sigResized.connect(self.updateViews)
        
        # Setup dynamic plotting process
        self.isGraphing = False
        self.timer = pg.QtCore.QTimer()
        self.timer.timeout.connect(self.updateUI) 
        self.timer.start(self.MIN_PLOT_UPDATE_PERIOD)  
        
        # Set GPIO mode to board so things go smoothly with RPi upgrades
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(5, GPIO.IN)
        GPIO.add_event_detect(5, GPIO.BOTH, callback=self.hpVsO2Changed, bouncetime=500)
        GPIO.setup(13, GPIO.IN)
        GPIO.add_event_detect(13, GPIO.BOTH, callback=self.triggerChanged, bouncetime=1)
        
        GPIO.setup(19, GPIO.IN)
        GPIO.add_event_detect(19, GPIO.RISING, callback=self.heartBeatDetected, bouncetime=1)
        
        self.nitrogenModeOn = GPIO.input(5)
        
    def updateViews(self):
        self.heartrateSlowPlot.setGeometry(self.pressureSlowPlot.getViewBox().sceneBoundingRect())
        self.heartrateSlowPlot.linkedViewChanged(self.pressureSlowPlot.getViewBox(),self.heartrateSlowPlot.XAxis)
        self.temperatureViewbox.setGeometry(self.pressureSlowPlot.getViewBox().sceneBoundingRect())
        self.temperatureViewbox.linkedViewChanged(self.pressureSlowPlot.getViewBox(),self.temperatureViewbox.XAxis)
        
    def heartBeatDetected(self, chan):
       trig_time = time.time() - self.start_time
       self.HEART_BEATS_COUNTED = self.HEART_BEATS_COUNTED + 1
       self.heartBeatArray[self.heartBeatIdx] = trig_time;
       if(self.heartBeatIdx == (self.HEART_BEATS_TO_AVG-1)):
         self.heartBeatIdx = 0 #Reset
       else:
         self.heartBeatIdx = self.heartBeatIdx + 1
       # Update heartBeat line
       self.heartBeatLine_Idx += 1
       while (self.heartBeatLine_Idx >= self.MAX_HEARTBEAT_DISP):
         self.heartBeatLine_Idx -= self.MAX_HEARTBEAT_DISP
       self.heartBeatLines[self.heartBeatLine_Idx].setValue(trig_time)
      
    def triggerChanged(self,chan):
       # Update next trigger line
       trig_time = time.time() - self.start_time
       self.triggerIdx += 1
       while (self.triggerIdx >= self.MAX_TRIGGERS_DISP):
         self.triggerIdx -= self.MAX_TRIGGERS_DISP
       self.triggerLines[self.triggerIdx].setValue(trig_time)
       
    def hpVsO2Changed(self,chan):
       time.sleep(0.5)
       self.nitrogenModeOn = GPIO.input(5)
       
    def updateSlowPlotRefreshRate(self):
       self.NEXT_SLOW_UPDATE = self.NEXT_SLOW_UPDATE-self.SLOW_UPDATE_PERIOD+self.ui.slowUpdatePeriod.value()
       self.SLOW_UPDATE_PERIOD = self.ui.slowUpdatePeriod.value()
     
    def updatePlotTimeRange(self):
        self.PLOT_TIME_RANGE =  self.ui.fastUpdatePeriod.value()
        
        # Update plotting size
        old_queue_size = self.QUEUE_SIZE
        new_queue_size = int(round(2*self.PLOT_TIME_RANGE/self.MIN_DATA_FETCH_PERIOD))
        copySize = min(old_queue_size, new_queue_size)
                
        new_time_queue = np.zeros(new_queue_size)
        new_canula_queue = np.zeros(new_queue_size)
        new_ecg_queue = np.zeros(new_queue_size)
        #new_temp_queue = np.zeros(new_queue_size)
        
        new_time_queue[0:(copySize-1)] = self.time_queue[0:(copySize-1)] 
        new_canula_queue[0:(copySize-1)] = self.canula_queue[0:(copySize-1)]  
        new_ecg_queue[0:(copySize-1)] = self.ecg_queue[0:(copySize-1)] 
        #new_temp_queue[0:(copySize-1)] = self.temp_queue[0:(copySize-1)] 
         
        self.time_queue = new_time_queue
        self.canula_queue = new_canula_queue
        self.ecg_queue = new_ecg_queue
        #self.temp_queue = new_temp_queue
        
        # Update buffer size
        self.QUEUE_SIZE = new_queue_size
        
        # Update axes
        self.maxXlim = self.minXlim + self.PLOT_TIME_RANGE
        self.ui.pressurePlot.setXRange(self.minXlim,self.maxXlim,padding=0)
        
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
        nDataToRead = np.int32(self.dataFetcher.sync_bufferLength.value);
        lastIdx = self.dataFetcher.sync_bufferEndIdx.value
        endRead = lastIdx + 1
        if(self.dataFetcher.sync_bufferFull.value):
          startRead = 0
        else:
          startRead = self.dataFetcher.sync_bufferStartIdx.value
        stopReading = min(startRead+nDataToRead,self.dataFetcher.BUFFERSIZE)
        self.dataFetcher.indexLock.release()
        
        # We can now copy from the buffer safely.
        if(nDataToRead > 0):
          copyIdx = 0
          for i in range(startRead,stopReading): 
              self.time_queue[copyIdx] = self.dataFetcher.sync_time_buf[i]
              self.canula_queue[copyIdx] = self.canulaPressureSlope*self.dataFetcher.sync_canula_buf[i]+self.canulaPressureIntercept
              self.ecg_queue[copyIdx] = self.dataFetcher.sync_ecg_buf[i]
              #self.temp_queue[copyIdx] = self.dataFetcher.sync_temp_buf[i]
              copyIdx += 1
        
          # In case data has wrapped... read the wrapped data too
          if(endRead <= startRead):
            for i in range(0,endRead):
              self.time_queue[copyIdx] = self.dataFetcher.sync_time_buf[i]
              self.canula_queue[copyIdx] = self.canulaPressureSlope*self.dataFetcher.sync_canula_buf[i]+self.canulaPressureIntercept
              self.ecg_queue[copyIdx] = self.dataFetcher.sync_ecg_buf[i]
              #self.temp_queue[copyIdx] = self.dataFetcher.sync_temp_buf[i]
              copyIdx += 1
          
          # Data is copied, so move pointer to free up buffer space under lock
          self.dataFetcher.indexLock.acquire()
          self.dataFetcher.sync_bufferStartIdx.value = endRead
          self.dataFetcher.sync_bufferLength.value = self.dataFetcher.sync_bufferLength.value - nDataToRead 
          #self.dataFetcher.sync_bufferFull.value = 0
          self.dataFetcher.indexLock.release()
    
          # Now we can take all the time we want to plot the data; the 
          # fetching process will keep taking new data while this slow plot 
          # operation occurs. Nevertheless, we try to update the plotted 
          # line quickly 
        
          # Roll data to put newest data last
          self.time_queue = np.roll(self.time_queue,-nDataToRead)
          self.canula_queue = np.roll(self.canula_queue,-nDataToRead)
          self.ecg_queue = np.roll(self.ecg_queue,-nDataToRead)
          #self.temp_queue = np.roll(self.temp_queue,-nDataToRead)
                
          # Show the new data
          self.pressureLine.setData(self.time_queue,self.canula_queue)
          self.ecgLine.setData(self.time_queue,self.ecg_queue)
          
        # Update time axis if necessary
        if(elapsed_time > self.maxXlim):
            # Update axes
            while(elapsed_time > self.maxXlim):
              self.minXlim += self.TIME_SHIFT_PCT*self.PLOT_TIME_RANGE
              self.maxXlim = self.minXlim + self.PLOT_TIME_RANGE
            self.ui.pressurePlot.setXRange(self.minXlim,self.maxXlim,padding=0) 
            self.ui.ecgPlot.setXRange(self.minXlim,self.maxXlim,padding=0) 
            
        # Update text and slow graph
        if(elapsed_time > min(self.NEXT_TEXT_UPDATE,self.NEXT_SLOW_UPDATE)):
          min_val = min(self.canula_queue)
          max_val = max(self.canula_queue)

          # Calculate pressures
          oxygen_pressure = self.regulatorPressureSlope*self.dataFetcher.getDataFromChannel(1)+self.regulatorPressureIntercept
          nitrogen_pressure = self.regulatorPressureSlope*self.dataFetcher.getDataFromChannel(2)+self.regulatorPressureIntercept
          hpGas_pressure = self.regulatorPressureSlope*self.dataFetcher.getDataFromChannel(3)+self.regulatorPressureIntercept
        
          # Calculate volumes
          oxygen_volume = oxygen_pressure*self.oxygenVolumeSlope + self.oxygenVolumeIntercept
          nitrogen_volume = nitrogen_pressure*self.nitrogenVolumeSlope + self.nitrogenVolumeIntercept
          hpGas_volume = hpGas_pressure*self.hpGasVolumeSlope + self.hpGasVolumeIntercept
          
          tidal_vol = oxygen_volume
          if(self.nitrogenModeOn):
            tidal_vol += nitrogen_volume
            mode_string = "Mode: Nitrogen & Oxygen"
          else:
            tidal_vol += hpGas_volume
            mode_string = "Mode: HP Gas & Oxygen"
          
          if(elapsed_time > self.NEXT_SLOW_UPDATE):
            while(elapsed_time > self.NEXT_SLOW_UPDATE):
                self.NEXT_SLOW_UPDATE += self.SLOW_UPDATE_PERIOD
          
          if(elapsed_time > self.NEXT_TEXT_UPDATE):
            while(elapsed_time > self.NEXT_TEXT_UPDATE):
                self.NEXT_TEXT_UPDATE += self.TEXT_UPDATE_PERIOD
            self.slow_time_queue.append(elapsed_time)
            self.slow_minPressure_queue.append(min_val)
            self.slow_maxPressure_queue.append(max_val)
            self.slow_tidalVolume_queue.append(tidal_vol)
            self.minPressureLine.setData(self.slow_time_queue,self.slow_minPressure_queue)
            self.maxPressureLine.setData(self.slow_time_queue,self.slow_maxPressure_queue)
            #self.tidalVolumeLine.setData(self.slow_time_queue,self.slow_tidalVolume_queue)
            #print "TIDAL VOLS:", self.slow_tidalVolume_queue
            
            self.ui.nitrogenText.setPlainText("Nitrogen\nP: %3.1f psi  V: %4.2f mL" % (nitrogen_pressure,nitrogen_volume)) 
            self.ui.oxygenText.setPlainText("Oxygen\nP: %3.1f psi  V: %4.2f mL" % (oxygen_pressure,oxygen_volume)) 
            self.ui.hpText.setPlainText("HP Gas\nP: %3.1f psi  V: %4.2f mL" % (hpGas_pressure,hpGas_volume)) 
            
            self.ui.canulaText.setPlainText("Canula\nPmax: %3.1f cmH20\nPmin: %3.1f cmH20\nTV  : %4.2f mL" % (max_val, min_val,tidal_vol))  
            self.ui.modeText.setPlainText(mode_string)    

            # Crude check that animal is alive
            timeSinceLastRecordedHeartBeat = (time.time() - self.start_time - self.heartBeatArray[self.heartBeatIdx -1])
            if(timeSinceLastRecordedHeartBeat > self.MAX_TIME_BETWEEN_HEARTBEATS):
               self.HEART_BEATS_COUNTED = 0
               self.heartBeatArray = [0]*self.HEART_BEATS_TO_AVG
               self.heartBeatIdx = 0 #Reset
            
            if(self.HEART_BEATS_COUNTED == 0):
                 self.ui.heartRateText.setPlainText("Heart Rate: NOT DETECTED");
            elif(self.HEART_BEATS_COUNTED < self.HEART_BEATS_TO_AVG):
                self.ui.heartRateText.setPlainText("Heart Rate: Calculating");
            else:
                # Calculate and update heart rate
                timeForAllBeats = (max(self.heartBeatArray) - min(self.heartBeatArray))
                if(timeForAllBeats > 0):
                  
                  heartRate = 60*(self.HEART_BEATS_TO_AVG-1) / timeForAllBeats
                  #print "HEART BEATS:"
                  #print self.heartBeatArray
                  #print "MAX: %5.3f  MIN: %5.3f" % (max(self.heartBeatArray), min(self.heartBeatArray))
                  #print "TIME FOR ALL BEATS: %5.3f" % timeForAllBeats
                  #print "heartRate = %8.3f" % heartRate
                  #print "+++++++++++++++++++++++++"

                  heart_rate_string = "Heart Rate: %4.1f BPM" % heartRate
                  self.ui.heartRateText.setPlainText(heart_rate_string);

                  self.slow_heartRate_queue.append(heartRate)
                  self.slow_timeHR_queue.append(elapsed_time)
                  self.heartRateLine.setData(self.slow_timeHR_queue,self.slow_heartRate_queue)
                  
            
            # Calculate Temperatures            
            bore_temp = np.polyval(self.tempSensor_fit,self.dataFetcher.getDataFromChannel(7))
            animal_temp = np.polyval(self.tempSensor_fit,self.dataFetcher.getDataFromChannel(6))
            tempTextString = "Bore Temp: %4.1f\nAnimal Temp: %4.1f" % (bore_temp, animal_temp)
            self.ui.temperatureText.setPlainText(tempTextString)
                    
    def closeEvent(self, ce):
        self.stopGraphing()
        self.dataFetcher.stopFetching()    
        

## Start Qt event loop unless running in interactive mode.
if __name__ == '__main__':
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        app = QtGui.QApplication([])
        win = DataMonitoringWindow()
        win.show()
        win.startGraphing()
        sys.exit(app.exec_())
