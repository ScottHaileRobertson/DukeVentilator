#!/usr/bin/python
import time
import multiprocessing as mp 
import numpy as np
import csv
#import spidev

class TimedDataFetcher:
  def __init__(self, buffersize, fetchperiod):
    self.start_time = time.time()
    self.BUFFERSIZE = buffersize
    self.FETCHPERIOD = fetchperiod
    self.isFetching = False
    self.spi = []
    
    print "BUFFERSIZE = %d" % self.BUFFERSIZE
    
    # Create lock to prevent clashes between graphing/fetching processes
    self.lock = mp.Lock()

    # Create dummy csv file
    #self.csvfile = []
    #self.csvwriter = []

    # Setup an array to buffer values that are read off the pressure monitors 
    self.lock.acquire()
    
    self.sync_time_buf = mp.Array('d',range(self.BUFFERSIZE))
    self.sync_canula_buf = mp.Array('d',range(self.BUFFERSIZE))
    #self.sync_o2_buf = mp.Array('d',range(self.BUFFERSIZE))
    #self.sync_n2_buf = mp.Array('d',range(self.BUFFERSIZE))
    #self.sync_hp_buf = mp.Array('d',range(self.BUFFERSIZE))
    #self.sync_ecg_buf = mp.Array('d',range(self.BUFFERSIZE))
    #self.sync_temp_buf = mp.Array('d',range(self.BUFFERSIZE))
    self.sync_trig_buf = mp.Array('d',range(self.BUFFERSIZE))
    
    self.sync_bufferStartIdx = mp.Value('I', 0)
    self.sync_bufferLength = mp.Value('I', 0)
    self.sync_bufferEndIdx = mp.Value('I', -1)
    self.sync_bufferFull = mp.Value('I', 0)  # SHOULD BE BOOLEAN
    
    self.lock.release() 
    
    self.fetch_process = mp.Process(target=self.fetchData, args=(), 
                           name='FetchProcess')
                           
  def newStartTime(self,newStartTime):
    print("UPDATING START TIME")
    self.lock.acquire()
    self.start_time = newStartTime
    self.sync_time_buf = mp.Array('d',range(self.BUFFERSIZE))
    self.sync_canula_buf = mp.Array('d',range(self.BUFFERSIZE))
    #self.sync_o2_buf = mp.Array('d',range(self.BUFFERSIZE))
    #self.sync_n2_buf = mp.Array('d',range(self.BUFFERSIZE))
    #self.sync_hp_buf = mp.Array('d',range(self.BUFFERSIZE))
    #self.sync_ecg_buf = mp.Array('d',range(self.BUFFERSIZE))
    #self.sync_temp_buf = mp.Array('d',range(self.BUFFERSIZE))
    self.sync_trig_buf = mp.Array('d',range(self.BUFFERSIZE))
    
    self.sync_bufferStartIdx = mp.Value('I', 0)
    self.sync_bufferLength = mp.Value('I', 0)
    self.sync_bufferEndIdx = mp.Value('I', -1)
    self.sync_bufferFull = mp.Value('I', 0)  # SHOULD BE BOOLEAN
    
    self.lock.release()         
        
  def updateBufferSize(self, newBufferSize):
      print("UPDATING BUFFER SIZE from %d to %d" % (self.BUFFERSIZE, newBufferSize))
      
      # Do everything under lock
      self.lock.acquire()
      
      self.BUFFERSIZE = newBufferSize
      self.sync_time_buf = mp.Array('d',range(self.BUFFERSIZE))
      self.sync_canula_buf = mp.Array('d',range(self.BUFFERSIZE))
      #self.sync_o2_buf = mp.Array('d',range(self.BUFFERSIZE))
      #self.sync_n2_buf = mp.Array('d',range(self.BUFFERSIZE))
      #self.sync_hp_buf = mp.Array('d',range(self.BUFFERSIZE))
      #self.sync_ecg_buf = mp.Array('d',range(self.BUFFERSIZE))
      #self.sync_temp_buf = mp.Array('d',range(self.BUFFERSIZE))
      self.sync_trig_buf = mp.Array('d',range(self.BUFFERSIZE))
      
      self.sync_bufferStartIdx.value = 0
      self.sync_bufferLength.value = 0
      self.sync_bufferEndIdx.value = -1
      self.sync_bufferFull.value = 0  # SHOULD BE BOOLEAN
      
      # free up the lock 
      self.lock.release()
      
      print "BUFFER SIZE UPDATED."
      
          
  def getDataFromChannel(self, channel):
      #adc = self.spi.xfer2([1,(8+channel)<<4,0]) 
      #return ((adc[1]&3) << 8) + adc[2]
      return 100*np.random.random_sample()
      
  def fetchData(self):  
    # Open SPI bus
    #self.spi = spidev.SpiDev()
    #self.spi.open(0,0)
  
    # Fetch new data until the end of time, or when the user closes the window
    while self.isFetching:
      # Get timestamp for data
      print "FETCHING NEW DATA!"
      t_stamp = time.time() - self.start_time
    
      # Fetch new data
      
      canula = self.getDataFromChannel(0)
      o2 = self.getDataFromChannel(1)
      n2 = self.getDataFromChannel(2)
      hp = self.getDataFromChannel(3)
      trig = self.getDataFromChannel(4)
      ecg = self.getDataFromChannel(6)
      temp = self.getDataFromChannel(5)
      
    
      # Add data to buffer and increment index under lock
      self.lock.acquire()
      self.sync_bufferEndIdx.value += 1
      self.sync_bufferLength.value += 1
      
      if(self.sync_bufferLength.value >= (self.BUFFERSIZE)):
        # Buffer has overrun! (will be reset when/if plotting catches up)
        self.sync_bufferFull.value = 1
        self.sync_bufferLength.value = self.BUFFERSIZE
        print "   BUFFER FULL !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
        
      # Buffer has space, so add new data
      if(self.sync_bufferEndIdx.value >= self.BUFFERSIZE):
        print "   BUFFER wraps... unwrapping"
        while(self.sync_bufferEndIdx.value >= self.BUFFERSIZE):
          self.sync_bufferEndIdx.value = self.sync_bufferEndIdx.value - self.BUFFERSIZE
      print "   Incremented buffer end to %d" % (self.sync_bufferEndIdx.value)
      print "   Incremented buffer length to %d" % (self.sync_bufferLength.value)
      self.lock.release()
      
      print "   setting self.sync_time_buf[%d] = %d" % (self.sync_bufferEndIdx.value, t_stamp)
      print "   setting self.sync_canula_buf[%d] = %d" % (self.sync_bufferEndIdx.value, canula)
      print "   setting self.sync_trig_buf[%d] = %d" % (self.sync_bufferEndIdx.value, trig)
      self.sync_time_buf[self.sync_bufferEndIdx.value] = t_stamp
      self.sync_canula_buf[self.sync_bufferEndIdx.value] = canula
      #self.sync_o2_buf[self.sync_bufferEndIdx.value] = o2
      #self.sync_n2_buf[self.sync_bufferEndIdx.value] = n2
      #self.sync_hp_buf[self.sync_bufferEndIdx.value] = hp
      #self.sync_ecg_buf[self.sync_bufferEndIdx.value] = ecg
      #self.sync_temp_buf[self.sync_bufferEndIdx.value = temp
      self.sync_trig_buf[self.sync_bufferEndIdx.value] = trig

      # Write to csvfile
      #self.csvwriter.writerow([t_stamp,canula,o2,n2,hp,ecg,temp,trig])
      print "FETCHED NEW DATA."
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
  

