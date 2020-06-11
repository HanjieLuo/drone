import calendar
import atexit
import time
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
from scipy import linalg
import sys
import os
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(
    os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))


class MagnetometerCalibration:
    def __init__(self):
        self.mag_x = list()
        self.mag_y = list()
        self.mag_z = list()
        self.plot = None
        self.project_root = os.path.dirname(os.path.realpath(__file__))
        time.sleep(0.5)

    def RecordMagData(self):
        from com_task import ComTask
        self.com_task = ComTask("COM5", 115200)
        self.com_task.start()

        ts = calendar.timegm(time.gmtime())
        mag_file = open(self.project_root +
                        '/data/mag_cali/drone_mag_' + str(ts) + '.txt', 'w')

        def cleanup():
            print("Exit")
            mag_file.close()

        atexit.register(cleanup)

        fig = plt.figure(figsize=(10, 10))
        self.ax = fig.add_subplot(111, projection='3d')
        while True:
            [timestamp, xacc, yacc, zacc, xgyro, ygyro, zgyro,
                xmag, ymag, zmag] = self.com_task.getData()
            print('%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f' % (
                timestamp, xacc, yacc, zacc, xgyro, ygyro, zgyro, xmag, ymag, zmag))

            mag_data = '%f %f %f\n' % (xmag, ymag, zmag)
            mag_file.write(mag_data)

            self.mag_x.append(xmag)
            self.mag_y.append(ymag)
            self.mag_z.append(zmag)

            self.ax.cla()
            self.ax.set_xlim(-1000, 1000)
            self.ax.set_ylim(-1000, 1000)
            self.ax.set_zlim(-1000, 1000)
            self.plot = self.ax.scatter(
                self.mag_x, self.mag_y, self.mag_z, marker='o', color='g')

            plt.pause(0.01)  # 设置时间间隔
        plt.show()

    def ShowMagData(self, path):
        data_mag = np.loadtxt(self.project_root + path, dtype=np.float32)
        mag_x = data_mag[:, 0].tolist()
        mag_y = data_mag[:, 1].tolist()
        mag_z = data_mag[:, 2].tolist()

        fig = plt.figure(figsize=(10, 10))
        ax = fig.add_subplot(111, projection='3d')
        ax.set_xlim(-1000, 1000)
        ax.set_ylim(-1000, 1000)
        ax.set_zlim(-1000, 1000)
        ax.scatter(mag_x, mag_y, mag_z, marker='o', color='g')
        plt.show()
    
    def Calibrate(self, path):
        data_mag = np.loadtxt(self.project_root + path, dtype=np.float32)
        mag_x = data_mag[:, 0]
        mag_y = data_mag[:, 1]
        mag_z = data_mag[:, 2]
        
        Q, n, d = self.ellipsoid_fitting(mag_x, mag_y, mag_z)
        
        print(Q)
        print(n)
        print(d)
        
        Qinv = np.linalg.inv(Q)
        print(Qinv)
        b = - 0.5 * np.dot(Qinv, n)
        print(b)
        alpha = 1.0 / (0.25 * np.dot(n.T, np.dot(Qinv, n)) - d)
        print(alpha)
        Ainv = np.real(np.sqrt(alpha) * linalg.sqrtm(Q))
        print(Ainv)


    def ellipsoid_fitting(self, X, Y, Z):
        a1 = X ** 2
        a2 = Y ** 2
        a3 = Z ** 2
        a4 = 2 * np.multiply(Y, Z)
        a5 = 2 * np.multiply(X, Z)
        a6 = 2 * np.multiply(X, Y)
        a7 = 2 * X
        a8 = 2 * Y
        a9 = 2 * Z
        a10 = np.ones(len(X)).T
        D = np.array([a1, a2, a3, a4, a5, a6, a7, a8, a9, a10])

        # Eqn 7, k = 4
        C1 = np.array([[-1, 1, 1, 0, 0, 0],
                       [1, -1, 1, 0, 0, 0],
                       [1, 1, -1, 0, 0, 0],
                       [0, 0, 0, -4, 0, 0],
                       [0, 0, 0, 0, -4, 0],
                       [0, 0, 0, 0, 0, -4]])

        # Eqn 11
        S = np.matmul(D, D.T)
        S11 = S[:6, :6]
        S12 = S[:6, 6:]
        S21 = S[6:, :6]
        S22 = S[6:, 6:]

        # Eqn 15, find eigenvalue and vector
        # Since S is symmetric, S12.T = S21
        tmp = np.matmul(np.linalg.inv(C1), S11 - np.matmul(S12,
                                                           np.matmul(np.linalg.inv(S22), S21)))
        eigenValue, eigenVector = np.linalg.eig(tmp)
        u1 = eigenVector[:, np.argmax(eigenValue)]

        # Eqn 13 solution
        u2 = np.matmul(-np.matmul(np.linalg.inv(S22), S21), u1)

        # Total solution
        u = np.concatenate([u1, u2]).T

        Q = np.array([[u[0], u[5], u[4]],
                      [u[5], u[1], u[3]],
                      [u[4], u[3], u[2]]])

        n = np.array([[2 * u[6]],
                      [2 * u[7]],
                      [2 * u[8]]])

        d = u[9]

        return Q, n, d


if __name__ == "__main__":
    mag_cali = MagnetometerCalibration()
    # mag_cali.RecordMagData()
    # mag_cali.ShowMagData("/data/drone_mag_1590423949.txt")
    mag_cali.Calibrate("/data/drone_mag_1590423949.txt")
