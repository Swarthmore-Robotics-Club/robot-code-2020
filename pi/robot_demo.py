#!/usr/bin/env python   
  
import time
import serial
import struct
import math
import sys

if len(sys.argv) >= 2:
    pattern = sys.argv[1]
else:
    pattern = 'sin'

RPI_COMM_MESSAGE_KP_A = 2
RPI_COMM_MESSAGE_KI_A = 3
RPI_COMM_MESSAGE_KD_A = 4
RPI_COMM_MESSAGE_KCALIB_A = 5
RPI_COMM_MESSAGE_KP_B = 6
RPI_COMM_MESSAGE_KI_B = 7
RPI_COMM_MESSAGE_KD_B = 8
RPI_COMM_MESSAGE_KCALIB_B = 9

ser = serial.Serial(            
 port='/dev/ttyS0',
 baudrate = 9600,
 parity=serial.PARITY_NONE,
 stopbits=serial.STOPBITS_ONE,
 bytesize=serial.EIGHTBITS,
 timeout=1
)

_filter_lvel = 0
_filter_rvel = 0
_filter_prev_t = time.clock_gettime(time.CLOCK_BOOTTIME)
_filter_max_step = 100.
def write_filter_velocities(ser, left, right):
    global _filter_lvel
    global _filter_rvel
    global _filter_prev_t
    global _filter_max_step
    ldiff = left - _filter_lvel
    rdiff = right - _filter_rvel
    t = time.clock_gettime(time.CLOCK_BOOTTIME)
    dt = t - _filter_prev_t
    _filter_prev_t = t
    ldiff = min(max(ldiff, -_filter_max_step * dt), _filter_max_step * dt)
    rdiff = min(max(rdiff, -_filter_max_step * dt), _filter_max_step * dt)
    _filter_lvel += ldiff
    _filter_rvel += rdiff
    write_velocities(ser, _filter_lvel, _filter_rvel)

def write_velocities(ser, left, right):
  ser.write(struct.pack('<bff', 1, right, left))

def write_pid_value(ser, pid_id, value):
  ser.write(struct.pack('<bf', pid_id, value))


prev_left_encoder = 0
prev_right_encoder = 0
left_encoder = 0
right_encoder = 0
_read_current_message = b''
_read_message_type = 0
def update_read(ser):
  global left_encoder
  global right_encoder
  global prev_left_encoder
  global prev_right_encoder
  global _read_current_message
  global _read_message_type
  if ser.in_waiting > 0:
    _read_current_message += ser.read(ser.in_waiting)
  while len(_read_current_message) > 0:
    if _read_message_type == 0:
      _read_message_type = _read_current_message[0]
      _read_current_message = _read_current_message[1:]
      continue
    if _read_message_type == 1:
      if len(_read_current_message) >= 4:
        prev_left_encoder = left_encoder
        left_encoder = struct.unpack('<f', _read_current_message[:4])[0]
        _read_message_type = 0
        _read_current_message = _read_current_message[4:]
      else:
        break
    elif _read_message_type == 2:
      if len(_read_current_message) >= 4:
        prev_right_encoder = right_encoder
        right_encoder = struct.unpack('<f', _read_current_message[:4])[0]
        _read_message_type = 0
        _read_current_message = _read_current_message[4:]
      else:
        break
    else:
      _read_message_type = 0

# Open a connection and turn off motors
ser.write(b'\0'*32)
ser.write(b'robotics')
ser.write(b'\0'*32)
ser.write(struct.pack('<bff', 1, 0., 0.))

#write_pid_value(ser, RPI_COMM_MESSAGE_KP_A, 0.0018)
#write_pid_value(ser, RPI_COMM_MESSAGE_KI_A, 0.002)
#write_pid_value(ser, RPI_COMM_MESSAGE_KD_A, 0.000005)
#write_pid_value(ser, RPI_COMM_MESSAGE_KCALIB_A, 0.00055)
#write_pid_value(ser, RPI_COMM_MESSAGE_KP_B, 0.0018)
#write_pid_value(ser, RPI_COMM_MESSAGE_KI_B, 0.002)
#write_pid_value(ser, RPI_COMM_MESSAGE_KD_B, 0.000005)
#write_pid_value(ser, RPI_COMM_MESSAGE_KCALIB_B, 0.00060)


write_pid_value(ser, RPI_COMM_MESSAGE_KP_A, 0.02)
write_pid_value(ser, RPI_COMM_MESSAGE_KI_A, 0.001)
write_pid_value(ser, RPI_COMM_MESSAGE_KD_A, 0.000)
write_pid_value(ser, RPI_COMM_MESSAGE_KCALIB_A, 0.007)
write_pid_value(ser, RPI_COMM_MESSAGE_KP_B, 0.02)
write_pid_value(ser, RPI_COMM_MESSAGE_KI_B, 0.001)
write_pid_value(ser, RPI_COMM_MESSAGE_KD_B, 0.000)
write_pid_value(ser, RPI_COMM_MESSAGE_KCALIB_B, 0.0073)

kp = 40.


def clip_float(f):
  return round(f * 100.) * 0.01

if pattern == 'sin':
  while 1:
    t = time.clock_gettime(time.CLOCK_BOOTTIME)
    write_filter_velocities(ser, kp*math.cos(t*4), kp*math.cos(t*4))
    update_read(ser)
    print('left {}\tright {}'.format(clip_float(left_encoder - prev_left_encoder), clip_float(right_encoder - prev_right_encoder)))
    time.sleep(0.05)
elif pattern == 'line':
  while 1:
    write_filter_velocities(ser, kp, kp)
    update_read(ser)
    print('left {}\tright {}'.format(clip_float((left_encoder - prev_left_encoder)*10), clip_float((right_encoder - prev_right_encoder)*10)))
    time.sleep(0.05)
elif pattern == 'circle':
  while 1:
    write_filter_velocities(ser, 0.5*kp, kp)
    time.sleep(0.05)
elif pattern == 'polygon':
  while 1:
    t = time.clock_gettime(time.CLOCK_BOOTTIME)
    cycle = math.fmod(t, 4)
    if cycle < 1:
      write_filter_velocities(ser, 0.5*kp, -0.5*kp)
    else:
      write_filter_velocities(ser, kp, kp)
    update_read(ser)
    print('left {}\tright {}'.format(clip_float((left_encoder - prev_left_encoder)*10), clip_float((right_encoder - prev_right_encoder)*10)))
    time.sleep(0.05)
elif pattern == 'watch':
  while 1:
    update_read(ser)
    print('left {}\tright {}'.format(clip_float((left_encoder - prev_left_encoder)*10), clip_float(right_encoder - prev_right_encoder)))
    time.sleep(0.05)
  
