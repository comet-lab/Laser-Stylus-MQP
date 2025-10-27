import serial
import struct
import time
import math

class Arduino:
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


def testRandomShutOff():
    import numpy as np
    #HACKY WAY TO DO RELATIVE IMPORTS 
    import sys, os
    sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))
    from python_src.DataAnalysis.data_storage import DataStorage
    # Load data from experiment to get the laserOn signal
    storageObj = DataStorage.load_obj("/home/nepacheco/laserExperiments/Slotine-Adaptive/agar2/2023-11-07/Slotine-Adaptive_2023-11-07_agar2_AcuCO2_AdaptiveGain-[1.e-04,5.e-06,1.e-04]_ReferenceGain-100.0_v9")
    laser_obj = Arduino()
    period = round((storageObj.time[0,1]-storageObj.time[0,0])*100)/100.0  # get period to nearest 0.01
    laserStatus = np.zeros(500)
    for r in range(45,51):
        laserStatus = np.concatenate((laserStatus,np.ones(r),np.zeros(10),np.ones(150),np.zeros(150)))
    laserStatus = np.concatenate((laserStatus,np.zeros(1000)))
    timeDuration = 65  #storageObj.time[0,-1]  # get final time of simulation
    elapsedTime = 0
    laserOffCounter = 0

    laser_obj.set_output(0)
    counter = 0
    print("\nFiring in 3 Seconds\n")
    
    time.sleep(3)
    t = time.time()
    while (elapsedTime) < timeDuration:
            if (time.time() - t) >= period:
                elapsedTime = elapsedTime + (time.time() - t)
                t = time.time()
                laserOn = bool(laserStatus[counter])
                # laserOn = bool(storageObj.laserOn[0,counter])
                # offDuration = 0.07
                # if period < offDuration:
                #     if laserOn == True:  # if laser is turned on, check if it has been off at least 0.02 seconds.
                #         if laserOffCounter < round(offDuration/period - 1) and laserOffCounter >=0: 
                #             laserOn = False
                #             laserOffCounter += 1
                #         else:
                #             laserOffCounter = -1
                #     else:
                #             laserOffCounter += 1
                laser_obj.set_output(laserOn)
                print("Laser Status: ",laserOn)
                counter += 1
    print("Elapsed Time", elapsedTime)
    print("Done with Test")
    laser_obj.set_output(0)


def flipFlop():
    import numpy as np
    laser_obj = Arduino()
    timeDuration = 20
    period = 0.01
    elapsedTime = 0
    laserOn = 0
    print("Starting flip flop in 3 seconds")
    time.sleep(3)
    print("Starting flip flop")
    laser_obj.set_output(laserOn)
    t = time.time()
    while (elapsedTime) < timeDuration:
            if (time.time() - t) >= period:
                elapsedTime = elapsedTime + (time.time() - t)
                t = time.time()
                laser_obj.set_output(laserOn)
                if laserOn == 0:
                    laserOn = 1
                else:
                    laserOn = 0
                # laserOn = np.random.randint(0,2)
    print("Flip flop finished")
    laser_obj.set_output(0)

if __name__=='__main__':
    # testRandomShutOff()
    laser_obj = Arduino()
    print("Laser Firing in 2 seconds")
    time.sleep(2)
    print("Laser Firing")
    laser_obj.set_output(1)
    # time.sleep(.15)
    # laser_obj.set_output(0)
    # print("Laser Off")