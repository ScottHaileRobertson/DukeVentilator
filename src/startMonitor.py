#!/usr/bin/python
from pyqtgraph.Qt import QtGui, QtCore
import numpy as np
import pyqtgraph as pg
from mainwidget import Ui_MainWidget
import datafetcher as df
import multiprocessing as mp 
import time

# Version
VERSION_STRING = "1.0"

class DataMonitoringWindow(QtGui.QWidget):
    def __init__(self):
        # Setup UI 
        QtGui.QWidget.__init__(self)
        self.ui = Ui_MainWidget()
        self.ui.setupUi(self)
        self.setWindowTitle("Duke Animal Monitor v" + VERSION_STRING)
        
        # CONSTANTS
        self.MIN_DATA_FETCH_PERIOD = 0.001
        self.MIN_PLOT_UPDATE_PERIOD = 0.01 # A little more than 60Hz
        self.PLOT_TIME_RANGE = 10 # Total seconds of display
        self.QUEUE_SIZE = int(round(self.PLOT_TIME_RANGE/self.MIN_DATA_FETCH_PERIOD))
        self.TIME_SHIFT_PCT = 0.9
        self.TEXT_UPDATE_PERIOD = 1
        self.NEXT_TEXT_UPDATE = 3
        self.SLOW_UPDATE_PERIOD = 5;
        self.NEXT_SLOW_UPDATE = 1 
        
        
        # Create data fetching process
        self.dataFetcher = df.TimedDataFetcher(self.QUEUE_SIZE, self.MIN_DATA_FETCH_PERIOD)
                
        # Setup queues for plotting data as all NaNs
        self.time_queue = np.empty(self.QUEUE_SIZE)
        self.canula_queue = np.empty(self.QUEUE_SIZE)
        #self.o2_queue = np.empty(self.QUEUE_SIZE)
        #self.n2_queue = np.empty(self.QUEUE_SIZE)
        #self.hp_queue = np.empty(self.QUEUE_SIZE)
        #self.ecg_queue = np.empty(self.QUEUE_SIZE)
        #self.temp_queue = np.empty(self.QUEUE_SIZE)
        self.trig_queue = np.empty(self.QUEUE_SIZE)

        # Setup for slow plot data
        self.slow_time_queue = []
        self.slow_minPressure_queue = []
        self.slow_maxPressure_queue = []
        
        self.ui.fastUpdatePeriod.valueChanged.connect(self.updatePlotTimeRange)
        self.ui.slowUpdatePeriod.valueChanged.connect(self.updateSlowPlotRefreshRate)
        
        # Initialize graphs
        self.ui.pressurePlot.setMouseEnabled(x=False, y=True)
        self.ui.pressurePlot.enableAutoRange(x=False,y=True)
        self.ui.pressurePlot.setLabel('left', "Pressure", units='psi')
        self.ui.pressurePlot.setLabel('bottom', "Time", units='sec')
        self.minXlim = 0
        self.updatePlotTimeRange()
        self.updateSlowPlotRefreshRate()
        self.maxXlim = self.minXlim + self.PLOT_TIME_RANGE
        self.ui.pressurePlot.setXRange(0,self.PLOT_TIME_RANGE,padding=0)
        
        # Initialize pressure line
        self.pressureLine = pg.PlotCurveItem(x=[],y=[], \
           pen=pg.mkPen({'color': "0FF"}),antialias=True)
        self.ui.pressurePlot.addItem(self.pressureLine)
        
        # Initialize trigger line
        self.triggerLine = pg.PlotCurveItem(x=[],y=[], \
           pen=pg.mkPen({'color': "F0F"}),antialias=True)
        self.ui.pressurePlot.addItem(self.triggerLine)
        
        # Initialize Min and Max pressure lines
        self.minPressureLine = pg.PlotCurveItem(x=[],y=[], \
           pen=pg.mkPen({'color': "FF0"}),antialias=True)
        self.ui.vitalsPlot.addItem(self.minPressureLine)
        self.maxPressureLine = pg.PlotCurveItem(x=[],y=[], \
           pen=pg.mkPen({'color': "5F0"}),antialias=True)
        self.ui.vitalsPlot.addItem(self.maxPressureLine)
        self.ui.vitalsPlot.setLabel('bottom', "Time", units='sec')
        self.ui.vitalsPlot.setLabel('left', "Pressure", units='psi')
        
        self.ui.pressurePlot.show()
        
        # Setup dynamic plotting process
        self.isGraphing = False
        self.timer = pg.QtCore.QTimer()
        self.timer.timeout.connect(self.updateUI)               
             
    def updateSlowPlotRefreshRate(self):
       self.NEXT_SLOW_UPDATE = self.NEXT_SLOW_UPDATE-self.SLOW_UPDATE_PERIOD+self.ui.slowUpdatePeriod.value()
       self.SLOW_UPDATE_PERIOD = self.ui.slowUpdatePeriod.value()
     
    def updatePlotTimeRange(self):
        self.PLOT_TIME_RANGE =  self.ui.fastUpdatePeriod.value()
        
        # Update plotting size
        old_queue_size = self.QUEUE_SIZE
        self.QUEUE_SIZE = int(round(self.PLOT_TIME_RANGE/self.MIN_DATA_FETCH_PERIOD))
        copySize = min(old_queue_size, self.QUEUE_SIZE)
        
        
        new_time_queue = np.empty(self.QUEUE_SIZE)
        new_canula_queue = np.empty(self.QUEUE_SIZE)
        #new_o2_queue = np.empty(self.QUEUE_SIZE)
        #new_n2_queue = np.empty(self.QUEUE_SIZE)
        #new_hp_queue = np.empty(self.QUEUE_SIZE)
        #new_ecg_queue = np.empty(self.QUEUE_SIZE)
        #new_temp_queue = np.empty(self.QUEUE_SIZE)
        new_trig_queue = np.empty(self.QUEUE_SIZE)
        
        new_time_queue[0:(copySize-1)] = self.time_queue[0:(copySize-1)] 
        new_canula_queue[0:(copySize-1)] = self.canula_queue[0:(copySize-1)]  
        #new_o2_queue[0:(copySize-1)] = self.o2_queue[0:(copySize-1)] 
        #new_n2_queue[0:(copySize-1)] = self.n2_queue[0:(copySize-1)] 
        #new_hp_queue[0:(copySize-1)] = self.hp_queue[0:(copySize-1)] 
        #new_ecg_queue[0:(copySize-1)] = self.ecg_queue[0:(copySize-1)] 
        #new_temp_queue[0:(copySize-1)] = self.temp_queue[0:(copySize-1)] 
        new_trig_queue[0:(copySize-1)] = self.trig_queue[0:(copySize-1)] 
         

        
        #self.time_queue = new_time_queue
        #self.canula_queue = new_canula_queue
        #self.o2_queue = new_o2_queue
        #self.n2_queue = new_n2_queue
        #self.hp_queue = new_hp_queue
        #self.ecg_queue = new_ecg_queue
        #self.temp_queue = new_temp_queue
        #self.trig_queue = new_trig_queue
        
        # Update buffer size
        #self.dataFetcher.updateBufferSize(self.QUEUE_SIZE)
        
        
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
          
        # Copy data from buffer under lock, then reset index to zero so new data
        # will be added to the beggining of the buffer again
        self.dataFetcher.lock.acquire()
        rollDist = np.int32(-self.dataFetcher.sync_idx.value)
        hasNewData = (self.dataFetcher.sync_idx.value > 0)
        if(hasNewData):
            # Copy new data to plot list
            for i in range(self.dataFetcher.sync_idx.value):        
                self.time_queue[i] = self.dataFetcher.sync_time_buf[i]
                self.canula_queue[i] = self.dataFetcher.sync_canula_buf[i]
                #self.o2_queue[i] = self.dataFetcher.sync_o2_buf[i]
                #self.n2_queue[i] = self.dataFetcher.sync_n2_buf[i]
                #self.hp_queue[i] = self.dataFetcher.sync_hp_buf[i]
                #self.ecg_queue[i] = self.dataFetcher.sync_ecg_buf[i]
                #self.temp_queue[i] = self.dataFetcher.sync_temp_buf[i]
                self.trig_queue[i] = self.dataFetcher.sync_trig_buf[i]
            self.dataFetcher.sync_idx.value = 0
        self.dataFetcher.lock.release()
            
        # Now we can take all the time we want to plot the data; the 
        # fetching process will keep taking new data while this slow plot 
        # operation occurs. Nevertheless, we try to update the plotted 
        # line quickly 
        if(hasNewData):
            # Roll data to put newest data last
            self.time_queue = np.roll(self.time_queue,rollDist)
            self.canula_queue = np.roll(self.canula_queue,rollDist)
            #self.o2_queue = np.roll(self.o2_queue,rollDist)
            #self.n2_queue = np.roll(self.n2_queue,rollDist)
            #self.hp_queue = np.roll(self.hp_queue,rollDist)
            #self.ecg_queue = np.roll(self.ecg_queue,rollDist)
            #self.temp_queue = np.roll(self.temp_queue,rollDist)
            self.trig_queue = np.roll(self.trig_queue,rollDist)
                
            # Show the new data
            self.pressureLine.setData(self.time_queue,self.canula_queue)
            self.triggerLine.setData(self.time_queue,self.trig_queue)
          
        # Update time axis if necessary
        if(elapsed_time > self.maxXlim):
            # Update axes
            while(elapsed_time > self.maxXlim):
              self.minXlim += self.TIME_SHIFT_PCT*self.PLOT_TIME_RANGE
              self.maxXlim = self.minXlim + self.PLOT_TIME_RANGE
            self.ui.pressurePlot.setXRange(self.minXlim,self.maxXlim,padding=0) 
            
        # Update text and slow graph
        if(elapsed_time > self.NEXT_SLOW_UPDATE):
            while(elapsed_time > self.NEXT_SLOW_UPDATE):
                self.NEXT_SLOW_UPDATE += self.SLOW_UPDATE_PERIOD
            min_val = min(self.canula_queue)
            max_val = max(self.canula_queue)
            self.slow_time_queue.append(elapsed_time)
            self.slow_minPressure_queue.append(min_val)
            self.slow_maxPressure_queue.append(max_val)
            self.minPressureLine.setData(self.slow_time_queue,self.slow_minPressure_queue)
            self.maxPressureLine.setData(self.slow_time_queue,self.slow_maxPressure_queue)
            self.ui.maxPressureText.setPlainText("Max Pres: %0.3f" % (max_val))
            self.ui.minPressureText.setPlainText("Min Pres: %0.3f" % (min_val)) 
        
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

