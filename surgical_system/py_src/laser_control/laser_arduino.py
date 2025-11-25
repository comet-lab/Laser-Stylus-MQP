import serial
import struct
import time
import math

class Laser_Arduino:
    def __init__(self, port='/dev/ttyACM0'):
        self.port = serial.Serial(baudrate=115200,
                                  timeout=1,
                                  interCharTimeout=0.1,
                                  bytesize = serial.EIGHTBITS,
                                  stopbits = serial.STOPBITS_ONE,
                                  parity = serial.PARITY_NONE,
                                  write_timeout = 0)
        self.MESSAGE_LENGTH = 2
        self.START_BYTE = b'A'
        self.port.port = port
        try:
            self.port.close()
            self.port.open()
        except serial.serialutil.SerialException as e:
            print(e)
    
    def read_message(self):
        return_bytes = self.port.read(self.MESSAGE_LENGTH)
        if(len(return_bytes) >= 2):
            result = struct.unpack('>BB', return_bytes)
            message = Message(0, result[1])
            return message
        return -1
    
    def _write(self, is_output, laser_on):
        start_byte = struct.pack('>c', self.START_BYTE)
        #shift output bit right one and combine with laserOn bit into one byte
        data_byte = (2 * is_output) + laser_on
        cmd_byte = struct.pack('>B', data_byte)
        try:
            self.port.write(start_byte + cmd_byte)
            self.port.flush()
        except:
            raise Exception

    def set_output(self, laser_on):
        self._write(1, laser_on)

    def get_laser_state(self):
        self.port.reset_input_buffer()
        self._write(0, 0)
        return self.read_message()

class Message:
    def __init__(self, is_output, laser_on):
        self.is_output = is_output
        self.laser_on = laser_on



if __name__=='__main__':
    # testRandomShutOff()
    laser_obj = Laser_Arduino()
    print("Laser Firing in 2 seconds")
    time.sleep(2)
    print("Laser Firing")
    laser_obj.set_output(0)
    # time.sleep(.15)
    # laser_obj.set_output(0)
    # print("Laser Off")