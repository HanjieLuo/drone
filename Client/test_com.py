# -*- coding: UTF-8 -*-
import serial.tools.list_ports

class Ser(object):
    def __init__(self):
        serial_ports = [i[0] for i in serial.tools.list_ports.comports()]
        print(serial_ports)
        
        self.port = serial.Serial(port=port, baudrate=115200, bytesize=8, parity=serial.PARITY_NONE,
                                  stopbits=serial.STOPBITS_ONE, timeout=1)

    def send_cmd(self, cmd):
        self.port.write(cmd.encode('utf-8'))

    def read_cmd(self):
        response = self.port.readline()
        return response.decode('utf-8')

    def read_num(self, num):
        response = self.port.read(num)
        return response
