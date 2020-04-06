#!E:/Python/Python38/python.exe
# -*- coding: UTF-8 -*-
import time
import pymavlink
from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink2



class Client:
    def __init__(self, port="COM3"):
        self.master = mavutil.mavlink_connection(port, baud=115200)

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
            rc_channel_values = [65535 for _ in range(8)]
            rc_channel_values[id - 1] = pwm
            self.master.mav.rc_channels_override_send(
                self.master.target_system,                # target_system
                self.master.target_component,             # target_component
                *rc_channel_values)                  # RC channel list, in microseconds.
            print("set_rc_channel_pwm: [", id, ", ", pwm, "]")
            return True
        
        return False


if __name__ == "__main__":
    client = Client("COM3")
    
    while(True):
        client.set_rc_channel_pwm(1, 1000)
        time.sleep(0.1)
    