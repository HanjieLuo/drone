# -*- coding: UTF-8 -*-
import serial
import threading
import time
import atexit
import calendar
import numpy as np
# import serial.tools.list_ports

class ComTask(threading.Thread):
    def __init__(self, port="COM5", baud=115200):
        threading.Thread.__init__(self, name="ComTask")
        self.daemon = True
        
        self.port = serial.Serial(port=port, baudrate=baud, bytesize=8, parity=serial.PARITY_NONE,
                                  stopbits=serial.STOPBITS_ONE, timeout=1)
        self.port.flushInput()
        self.port.flushOutput()
        
        self.buf = bytearray()
        self.data_string = None
    
    def record_data(self):
        ts = calendar.timegm(time.gmtime())
        acc_file = open('./data/drone_acc_' + str(ts) + '.txt', 'w')
        gyro_file = open('./data/drone_gyro_' + str(ts) + '.txt', 'w')
        
        def cleanup():
            print("Exit")
            acc_file.close()
            gyro_file.close()
        
        atexit.register(cleanup)
            
        while True:
            data = np.fromstring(self.ReadLine().decode('UTF-8'), dtype=float, sep=',')
            if data.shape[0] == 7:
                acc_data = '   %f   %f   %f   %f\n' % (data[0], data[1], data[2], data[3])
                gyro_data = '   %f   %f   %f   %f\n' % (data[0], data[4], data[5], data[6])
                acc_file.write(acc_data)
                gyro_file.write(gyro_data)
                print('%f\t%f\t%f\t%f\t%f\t%f\t%f' % (data[0], data[1], data[2], data[3], data[4], data[5], data[6]))

    def run(self):
        self.getIMU()

    def getIMU(self):
        while(True):
            self.data_string = self.ReadLine()
            time.sleep(0.001)
    
    def ReadLine(self):
        i = self.buf.find(b"\n")
        r = None
        if i >= 0:
            r = self.buf[:i+1]
            self.buf = self.buf[i+1:]
            return r
        while True:
            i = max(1, min(2048, self.port.in_waiting))
            # print(i)
            data = self.port.read(i)
            i = data.find(b"\n")
            if i >= 0:
                r = self.buf + data[:i+1]
                self.buf[0:] = data[i+1:]
                return r
            else:
                self.buf.extend(data)
    
    def getData(self):
        if self.data_string is not None:
            data = np.fromstring(self.data_string.decode('UTF-8'), dtype=float, sep=',')
            return data.tolist()
        
        return []
        
        
if __name__ == "__main__":
    com_task = ComTask("COM5", 115200)
    com_task.record_data()
    # com_task.start()
    # while(True):
    #     print(com_task.getData())

