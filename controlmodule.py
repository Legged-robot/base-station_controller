'''
Ben Katz
Motor Module Python API
Assumes the serial device is a nucleo running the firmware at:
Corresponding STM32F446 Firmware here:
https://os.mbed.com/users/benkatz/code/CanMaster/
'''
import serial
from struct import *
import time

class MotorModuleController():
    def __init__(self, port):
        try:
            self.ser = serial.Serial(port, 256000, timeout = 0.001) # timeout in s, 
            # self.ser = serial.Serial(port) # no timeout
            # self.ser.baudrate = 115200
            self.rx_data = 4*[(0.,)]
            self.tx_data = [0, 0, 0, 0, 0, 0, 0, 0, 0]
            print('connected to motor module controller')
        except:
            print('failed to connect to motor module controller')
            pass
    def send_data(self):
        pass
    def send_command(self, id, p_des, v_des, kp, kd, i_ff):
        """
        send_command(desired position, desired velocity, position gain, velocity gain, feed-forward current)
        Sends data over CAN, reads response, and populates rx_data with the response.
        """
        # Send command
        id = int(id)
        b = pack("f", p_des) + pack("f", v_des) + pack("f", kp) + pack("f", kd) + pack("f", i_ff) + bytes(bytearray([id])) 
        self.ser.write(b)
        # print(int.from_bytes(b, byteorder='big'))
        # Read the response
        self.ser.flushInput()
        b_rx = self.ser.read(13)

        if len(b_rx) == 13:
            self.rx_data[0] = unpack('f', b_rx[0:4])		# Position
            self.rx_data[1] = unpack('f', b_rx[4:8])		# Velocity
            self.rx_data[2] = unpack('f', b_rx[8:12])	    # Current
            self.rx_data[3] = b_rx[12];					    # ID
			# print(self.rx_data)
            return True
        else:
            # self.rx_data = 4*[(0.,)]
            return False
            print("Unsucessful read")

    def enable_motor(self, id):
        """
        Puts motor with CAN ID "id" into torque-control mode.  2nd red LED will turn on
        """
        b = b'\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFC'
        b = b + bytes(bytearray([id]))
        self.ser.write(b)
        #time.sleep(.1)
        #self.ser.flushInput()
    def disable_motor(self, id):
        """
        Removes motor with CAN ID "id" from torque-control mode.  2nd red LED will turn off
        """
        b = b'\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFD'
        b = b + bytes(bytearray([id]))
        self.ser.write(b)
    def zero_motor(self, id):
        """
        Sets motor with CAN ID "id" current position to be zero position 
        """
        b = b'\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFE'
        b = b + bytes(bytearray([id]))
        self.ser.write(b)