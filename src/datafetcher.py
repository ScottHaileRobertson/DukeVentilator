#!/usr/bin/python
import time
import multiprocessing as mp 
import numpy as np
import csv

class TimedDataFetcher:
  def __init__(self, buffersize, fetchperiod):
    self.start_time = time.time()
    self.BUFFERSIZE = buffersize
    self.FETCHPERIOD = fetchperiod
    self.isFetching = False
    
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
    self.sync_idx = mp.Value('I', 0)
    self.lock.release() 
    
    self.fetch_process = mp.Process(target=self.fetchData, args=(), 
                           name='FetchProcess')
                           
  def newStartTime(self,newStartTime):
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
    self.sync_idx.value = 0
    self.lock.release()         
        
  def updateBufferSize(self, newBufferSize):
      # Temporarily disable fetchng
      wasFetching = self.isFetching
      if wasFetching:
          self.endProcess()
      
      # Do everything under lock
      self.lock.acquire()
      
      # Start with empty buffer
      new_time_buff = np.empty(newBufferSize)
      new_canula_buff = np.empty(newBufferSize)
      #new_o2_buff = np.empty(newBufferSize)
      #new_n2_buff = np.empty(newBufferSize)
      #new_hp_buff = np.empty(newBufferSize)
      #new_ecg_buff = np.empty(newBufferSize)
      #new_temp_buff = np.empty(newBufferSize)
      new_trig_buff = np.empty(newBufferSize)
      
      # Roll/unroll to make the correct number of data at the beggining
      copySize = min(newBufferSize,self.BUFFERSIZE)
      rollAmount = copySize-self.sync_idx.value
      self.sync_time_buf = np.roll(self.sync_time_buf,rollAmount)
      self.sync_canula_buf = np.roll(self.sync_canula_buf,rollAmount)
      #self.sync_o2_buf = np.roll(self.sync_o2_buf,rollAmount)
      #self.sync_n2_buf = np.roll(self.sync_n2_buf,rollAmount)
      #self.sync_hp_buf = np.roll(self.sync_hp_buf,rollAmount)
      #self.sync_ecg_buf = np.roll(self.sync_ecg_buf,rollAmount)
      #self.sync_temp_buf = np.roll(self.sync_temp_buf,rollAmount)
      self.sync_trig_buf = np.roll(self.sync_trig_buf,rollAmount)      
      
      # Carefully copy the correct ammount of data from the old buffer
      new_time_buff = self.sync_time_buf[0:copySize]
      new_canula_buff = self.sync_canula_buf[0:copySize]
      #new_o2_buff = self.sync_o2_buf[0:copySize]
      #new_n2_buff = self.sync_n2_buf[0:copySize]
      #new_hp_buff = self.sync_hp_buf[0:copySize]
      #new_ecg_buff = self.sync_ecg_buf[0:copySize]
      #new_temp_buff = self.sync_temp_buf[0:copySize]
      new_trig_buff = self.sync_trig_buf[0:copySize]
                    
      # Update buffer
      self.sync_time_buf = new_time_buff
      self.sync_canula_buf = new_canula_buff
      #self.sync_o2_buf = new_o2_buff
      #self.sync_n2_buf = new_n2_buff
      #self.sync_hp_buf = new_hp_buff
      #self.sync_ecg_buf = new_ecg_buff
      #self.sync_temp_buf = new_temp_buff
      self.sync_trig_buf = new_trig_buff
      
      self.BUFFERSIZE = newBufferSize
      self.sync_idx.value = copySize
          
      # free up the lock 
      self.lock.release()
      
      # Start back up the data fetching with the new buffer size
      if wasFetching:
          self.startProcess()  
  def getDataFromChannel(self, channel):
      #adc = spi.xfer2([1,(8+channel)<<4,0]) 
      #return ((adc[1]&3) << 8) + adc[2]
      return 100*np.random.random_sample()
      
  def fetchData(self):  
    # Open SPI bus
    #spi = spidev.SpiDev()
    #spi.open(0,0)
  
    # Fetch new data until the end of time, or when the user closes the window
    while True:
      # Get timestamp for data
      t_stamp = time.time() - self.start_time
    
      # Fetch new data
      canula = self.getDataFromChannel(0)
      o2 = self.getDataFromChannel(1)
      n2 = self.getDataFromChannel(2)
      hp = self.getDataFromChannel(3)
      ecg = self.getDataFromChannel(4)
      temp = self.getDataFromChannel(5)
      trig = self.getDataFromChannel(6)
    
      # Add data to buffer and increment index under lock
      self.lock.acquire()
      if(self.sync_idx.value < (self.BUFFERSIZE - 1)):
        self.sync_time_buf[self.sync_idx.value] = t_stamp
        self.sync_canula_buf[self.sync_idx.value] = canula
        #self.sync_o2_buf[self.sync_idx.value] = o2
        #self.sync_n2_buf[self.sync_idx.value] = n2
        #self.sync_hp_buf[self.sync_idx.value] = hp
        #self.sync_ecg_buf[self.sync_idx.value] = ecg
        #self.sync_temp_buf[self.sync_idx.value] = temp
        self.sync_trig_buf[self.sync_idx.value] = trig
        self.sync_idx.value += 1
      else:
        self.sync_time_buf = np.roll(self.sync_time_buf,-1)
        self.sync_canula_buf = np.roll(self.sync_canula_buf,-1)
        #self.sync_o2_buf = np.roll(self.sync_o2_buf,-1)
        #self.sync_n2_buf = np.roll(self.sync_n2_buf,-1)
        #self.sync_hp_buf = np.roll(self.sync_hp_buf,-1)
        #self.sync_ecg_buf = np.roll(self.sync_ecg_buf,-1)
        #self.sync_temp_buf = np.roll(self.sync_temp_buf,-1)
        self.sync_trig_buf = np.roll(self.sync_trig_buf,-1)
        
        self.sync_time_buf[self.BUFFERSIZE - 1] = t_stamp
        self.sync_canula_buf[self.BUFFERSIZE - 1] = canula
        #self.sync_o2_buf[self.BUFFERSIZE - 1] = o2
        #self.sync_n2_buf[self.BUFFERSIZE - 1] = n2
        #self.sync_hp_buf[self.BUFFERSIZE - 1] = hp
        #self.sync_ecg_buf[self.BUFFERSIZE - 1] = ecg
        #self.sync_temp_buf[self.BUFFERSIZE - 1] = temp
        self.sync_trig_buf[self.BUFFERSIZE - 1] = trig    
      self.lock.release()
      
      # Write to csvfile
      #self.csvwriter.writerow([t_stamp,canula,o2,n2,hp,ecg,temp,trig])
      
      time.sleep(self.FETCHPERIOD) 
      
  def startFetching(self):
      if not self.isFetching:
          #self.csvfile = open('DukeVentilatorData.csv','w')
          #self.csvwriter = csv.writer(self.csvfile)
          self.fetch_process.start()
          self.isFetching = True
    
  def stopFetching(self):
      if self.isFetching:
          self.fetch_process.terminate()
          self.fetch_process.join()
          self.isRunning = False
          #self.csvfile.close()
  

