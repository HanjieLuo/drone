# -*- coding: UTF-8 -*-
import time
import pymavlink
import math
from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink2


class Client:
    def __init__(self, port="COM3", baud=115200):
        self.master = mavutil.mavlink_connection(port, baud=baud) # 115200
        self.g = 9.7833

    def run(self):
        # acc_file = open('drone_acc.txt', 'w')
        # gyro_file = open('drone_gyro.txt', 'w')
        while True:
            msg = self.master.recv_match(blocking=False)
            if msg is None:
                continue
            if msg.get_type() == 'RAW_IMU':
                self.timestamp = msg.time_usec / 1000.0
                self.xacc, self.yacc, self.zacc, self.xgyro, self.ygyro, self.zgyro, self.xmag, self.ymag, self.zmag = self.imu_raw2real(msg.xacc, msg.yacc, msg.zacc, msg.xgyro, msg.ygyro, msg.zgyro, msg.xmag, msg.ymag, msg.zmag)
                # acc_data = '   %f   %f   %f   %f\n' % (timestamp, xacc, yacc, zacc)
                # gyro_data = '   %f   %f   %f   %f\n' % (timestamp, xgyro, ygyro, zgyro) 
                # print(acc_data)
            elif msg.get_type() == "BAD_DATA":
                print("[Error] Got bad data: ", msg.data)
            time.sleep(0.001)
    
    def imu_raw2real(self, xacc, yacc, zacc, xgyro, ygyro, zgyro, xmag, ymag, zmag):
        xacc_real = - xacc * self.g / 16384.0
        yacc_real = - yacc * self.g / 16384.0
        zacc_real = - zacc * self.g / 16384.0
        
        xgyro_real = xgyro * 0.00106422515
        ygyro_real = ygyro * 0.00106422515
        zgyro_real = zgyro * 0.00106422515
        
        xmag_real = xmag / 1090.0
        ymag_real = ymag / 1090.0
        zmag_real = zmag / 1090.0
        
        return xacc_real, yacc_real, zacc_real, xgyro_real, ygyro_real, zgyro_real,  xmag_real, ymag_real, zmag_real
        
    def set_rc_channel_pwm(self, id, pwm=1500):
        """ Set RC channel pwm value
        Args:
            id (TYPE): Channel ID
            pwm (int, optional): Channel pwm value 1100-1900
        """
        if id < 1:
            print("Channel does not exist.")
            return False

        # The RAW values of the RC channels sent to the MAV to override info
        # received from the RC radio. A value of -1 means no change to that
        # channel. A value of 0 means control of that channel should be
        # released back to the RC radio. The standard PPM modulation is as
        # follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual
        # receivers/transmitters might violate this specification.
        if id < 9:
            rc_channel_values = [0 for _ in range(8)]
            rc_channel_values[id - 1] = pwm
            self.master.mav.rc_channels_override_send(
                self.master.target_system,                # target_system
                self.master.target_component,             # target_component
                *rc_channel_values)                  # RC channel list, in microseconds.
            print("set_rc_channel_pwm: [", id, ", ", pwm, "]")
            return True

        return False


if __name__ == "__main__":
    # client = Client("COM3")
    client = Client("COM5", 115200)
    client.run()

    # value = 0
    # client.set_rc_channel_pwm(2, 100)
    # while(True):
    #     client.set_rc_channel_pwm(4, value)
    #     value = value + 10
    #     if value > 2800:
    #         value = 0
    #     time.sleep(0.1)
