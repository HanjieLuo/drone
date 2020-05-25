import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time
import atexit
import calendar
from com_task import ComTask


class MagCali:
    def __init__(self):
        self.com_task = ComTask("COM3", 115200)
        self.com_task.start()

        self.mag_x = list()
        self.mag_y = list()
        self.mag_z = list()
        self.plot = None
        time.sleep(0.5)
    
    def RecordMagData(self):
        ts = calendar.timegm(time.gmtime())
        mag_file = open('./data/mag_cali/drone_mag_' + str(ts) + '.txt', 'w')
        
        def cleanup():
            print("Exit")
            mag_file.close()
        
        atexit.register(cleanup)

        fig = plt.figure(figsize=(10, 10))
        self.ax = fig.add_subplot(111, projection='3d')
        while True:
            [timestamp, xacc, yacc, zacc, xgyro, ygyro, zgyro, xmag, ymag, zmag] = self.com_task.getData()
            print('%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f' %(timestamp, xacc, yacc, zacc, xgyro, ygyro, zgyro, xmag, ymag, zmag))

            mag_data = '%f %f %f\n' % (xmag, ymag, zmag)
            mag_file.write(mag_data)

            self.mag_x.append(xmag)
            self.mag_y.append(ymag)
            self.mag_z.append(zmag)

            self.ax.cla()
            self.ax.set_xlim(-1000, 1000)
            self.ax.set_ylim(-1000, 1000)
            self.ax.set_zlim(-1000, 1000)
            self.plot = self.ax.scatter(self.mag_x, self.mag_y, self.mag_z, marker='o', color='g')
            
            plt.pause(0.01)  # 设置时间间隔
        plt.show()
    
    def ReadMagData(self):
        data_mag = np.loadtxt("data/mag_cali/drone_mag_1590423949.txt", dtype=np.float32)
        mag_x = data_mag[:,0].tolist()
        mag_y = data_mag[:,1].tolist()
        mag_z = data_mag[:,2].tolist()

        fig = plt.figure(figsize=(10, 10))
        ax = fig.add_subplot(111, projection='3d')
        ax.set_xlim(-1000, 1000)
        ax.set_ylim(-1000, 1000)
        ax.set_zlim(-1000, 1000)
        ax.scatter(mag_x, mag_y, mag_z, marker='o', color='g')
        plt.show()


if __name__ == "__main__":
    mag_cali = MagCali()
    # mag_cali.RecordMagData()
    mag_cali.ReadMagData()