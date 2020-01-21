#!/usr/bin/env python   
  
import time
import serial
import struct
           
ser = serial.Serial(            
 port='/dev/ttyS0',
 baudrate = 9600,
 parity=serial.PARITY_NONE,
 stopbits=serial.STOPBITS_ONE,
 bytesize=serial.EIGHTBITS,
 timeout=1
)

ser.write('robotics'.encode('utf-8'))
ser.write(struct.pack('<ff', 500., 500.))
time.sleep(5)
ser.write(struct.pack('<ff', 0., 0.))
