#!/usr/bin/python
import time
import multiprocessing as mp 
import numpy as np
import csv
import spidev
    
class TimedDataFetcher:
  def __init__(self, fetchperiod):
    self.start_time = time.time()
    self.BUFFERSIZE = 5000
    self.FETCHPERIOD = fetchperiod
    self.isFetching = False
    
    # Open SPI bus
    self.spi = spidev.SpiDev()
    self.spi.open(0,0)
    
    # Create lock to prevent clashes between graphing/fetching processes
    self.indexLock = mp.Lock()
 
    # Setup an array to buffer values that are read off the pressure monitors 
    self.indexLock.acquire()
    
    self.sync_time_buf = mp.Array('d',range(self.BUFFERSIZE))
    self.sync_canula_buf = mp.Array('d',range(self.BUFFERSIZE))
    self.sync_ecg_buf = mp.Array('d',range(self.BUFFERSIZE))
    #self.sync_temp_buf = mp.Array('d',range(self.BUFFERSIZE))
    
    self.sync_bufferStartIdx = mp.Value('I', 0)
    self.sync_bufferLength = mp.Value('I', 0)
    self.sync_bufferEndIdx = mp.Value('I', -1)
    self.sync_bufferFull = mp.Value('I', 0)  # SHOULD BE BOOLEAN
    self.indexLock.release() 
    
    self.fetch_process = mp.Process(target=self.fetchData, args=(), 
                           name='FetchProcess')
                           
  def newStartTime(self,newStartTime):
    self.indexLock.acquire()
    self.start_time = newStartTime
    self.sync_bufferStartIdx = mp.Value('I', 0)
    self.sync_bufferLength = mp.Value('I', 0)
    self.sync_bufferEndIdx = mp.Value('I', -1)
    self.sync_bufferFull = mp.Value('I', 0)  # SHOULD BE BOOLEAN
    self.indexLock.release()      
          
  def getDataFromChannel(self, channel):
      adc = self.spi.xfer2([1,(8+channel)<<4,0]) 
      return ((adc[1]&3) << 8) + adc[2]
      #return 100*np.random.random_sample()
      
  def fetchData(self):  
    # Fetch new data until the end of time, or when the user closes the window
    while self.isFetching:
      # Get timestamp for data
      t_stamp = time.time() - self.start_time
    
      # Fetch new data
      canula = self.getDataFromChannel(0)
      ecg = self.getDataFromChannel(5)
      #temp = self.getDataFromChannel(5)
      
      # Add data to buffer and increment index under lock
      self.indexLock.acquire()
      self.sync_bufferEndIdx.value += 1
      self.sync_bufferLength.value += 1
      
      if(self.sync_bufferLength.value >= (self.BUFFERSIZE)):
        # Buffer has overrun! (will be reset when/if plotting catches up)
        self.sync_bufferFull.value = 1
        self.sync_bufferLength.value = self.BUFFERSIZE
        print "   BUFFER FULL !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
        
      # Buffer has space, so add new data
      if(self.sync_bufferEndIdx.value >= self.BUFFERSIZE):
        while(self.sync_bufferEndIdx.value >= self.BUFFERSIZE):
          self.sync_bufferEndIdx.value = self.sync_bufferEndIdx.value - self.BUFFERSIZE
      self.indexLock.release()
      
       
      self.sync_time_buf[self.sync_bufferEndIdx.value] = t_stamp
      self.sync_canula_buf[self.sync_bufferEndIdx.value] = canula
      self.sync_ecg_buf[self.sync_bufferEndIdx.value] = ecg
      #self.sync_temp_buf[self.sync_bufferEndIdx.value = temp
      
      # Write to csvfile
      #self.csvwriter.writerow([t_stamp,canula,o2,n2,hp,ecg,temp,
            
      time.sleep(self.FETCHPERIOD) 
    
  def startFetching(self):
      if not self.isFetching:
          #self.csvfile = open('DukeVentilatorData.csv','w')
          #self.csvwriter = csv.writer(self.csvfile)
          self.isFetching = True
          self.fetch_process.start()
          
    
  def stopFetching(self):
      if self.isFetching:
          self.fetch_process.terminate()
          self.fetch_process.join()
          self.isFetching = False
          #self.csvfile.close()
