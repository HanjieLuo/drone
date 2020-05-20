# -*- coding: UTF-8 -*-
import serial
import threading
import time
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
        # serial_ports = [i[0] for i in serial.tools.list_ports.comports()]
        # print(serial_ports)

    def send(self, cmd):
        self.port.write(cmd.encode('utf-8'))

    def read(self):
        return self.port.readline()
        # response = self.port.readline()
        # return response.decode('utf-8')
    
    def run(self):
        self.getIMU()

    def getIMU(self):
        while(True):
            # self.port.read(self.port.in_waiting)                   
            # print(self.port.in_waiting)
            data = self.ReadLine()
            print(data)
            # time.sleep(0.01)
    
    def ReadLine(self):
        i = self.buf.find(b"\n")
        if i >= 0:
            r = self.buf[:i+1]
            self.buf = self.buf[i+1:]
            return r
        while True:
            i = max(1, min(2048, self.port.in_waiting))
            data = self.port.read(i)
            i = data.find(b"\n")
            if i >= 0:
                r = self.buf + data[:i+1]
                self.buf[0:] = data[i+1:]
                return r
            else:
                self.buf.extend(data)


if __name__ == "__main__":
    com_task = ComTask("COM5", 115200)
    com_task.start()
    while(True):
        pass

