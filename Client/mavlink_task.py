# -*- coding: UTF-8 -*-
import queue
import threading
import time
import math
import pymavlink
from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink2
import atexit
import calendar
import time


class MavkinkTask(threading.Thread):
    def __init__(self, port="COM3", baud=115200):
        threading.Thread.__init__(self, name="MavkinkTask")
        self.daemon = True

        self.imu_data = queue.Queue(1000)
        self.running = True

        self.master = mavutil.mavlink_connection(port, baud=baud)  # 115200
        self.g = 9.7833
        
        self.acc2real = self.g / 16384.0
        self.gyro2real = 0.00106422515
        self.mag2real = 0.00091743119

    def run(self):
        print("MavkinkTask Start...")
        self.running = True
        while self.running:
            try:
                # time.sleep(0.001)
                msg = self.master.recv_match(blocking=True)
                if msg is None:
                    continue
                if msg.get_type() == 'RAW_IMU':
                    timestamp = msg.time_usec / 1000.0
                    xacc, yacc, zacc, xgyro, ygyro, zgyro, xmag, ymag, zmag = self.imu_raw2real(
                        msg.xacc, msg.yacc, msg.zacc, msg.xgyro, msg.ygyro, msg.zgyro, msg.xmag, msg.ymag, msg.zmag)
                    # self.imu_data.put(
                    #     [timestamp, xacc, yacc, zacc, xgyro, ygyro, zgyro, xmag, ymag, zmag], block=False)
                    print('%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f' %
                          (timestamp, xacc, yacc, zacc, xgyro, ygyro, zgyro, xmag, ymag, zmag))
                elif msg.get_type() == "BAD_DATA" or msg.get_type() == 'ENCAPSULATED_DATA':
                    print("[Error] MavkinkTask got bad data: ", msg.data)
                    self.master.reset()

            except queue.Full:
                print("[Error] MavkinkTask: Queue.Full")
                # continue
            except Exception as e:
                # print(e)
                # self.running = False
                print("[Error] MavkinkTask: ", str(e))
                
    def stop(self):
        self.running = False
        
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
            # time.sleep(0.0001)
            msg = self.master.recv_match(blocking=False)
            if msg is None:
                continue
            if msg.get_type() == 'RAW_IMU':
                timestamp = msg.time_usec / 1000.0
                xacc, yacc, zacc, xgyro, ygyro, zgyro, xmag, ymag, zmag = self.imu_raw2real(
                    msg.xacc, msg.yacc, msg.zacc, msg.xgyro, msg.ygyro, msg.zgyro, msg.xmag, msg.ymag, msg.zmag)
                acc_data = '   %f   %f   %f   %f\n' % (timestamp, xacc, yacc, zacc)
                gyro_data = '   %f   %f   %f   %f\n' % (timestamp, xgyro, ygyro, zgyro)
                acc_file.write(acc_data)
                gyro_file.write(gyro_data)
                print('%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f' %
                      (timestamp, xacc, yacc, zacc, xgyro, ygyro, zgyro, xmag, ymag, zmag))
            elif msg.get_type() == "BAD_DATA" or msg.get_type() == 'ENCAPSULATED_DATA':
                print("[Error] MavkinkTask got bad data: ", msg.data)
                self.master.reset()
    

    def imu_raw2real(self, xacc, yacc, zacc, xgyro, ygyro, zgyro, xmag, ymag, zmag):
        xacc_real = - xacc * self.acc2real
        yacc_real = - yacc * self.acc2real
        zacc_real = - zacc * self.acc2real

        xgyro_real = xgyro * self.gyro2real
        ygyro_real = ygyro * self.gyro2real
        zgyro_real = zgyro * self.gyro2real

        xmag_real = xmag * self.mag2real
        ymag_real = ymag * self.mag2real
        zmag_real = zmag * self.mag2real

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
    mavlink_task = MavkinkTask("COM5", 115200)
    mavlink_task.record_data()
    # mavlink_task.start()
    # time.sleep(5)
    # mavlink_task.stop()
    # while(True):
    #     pass
