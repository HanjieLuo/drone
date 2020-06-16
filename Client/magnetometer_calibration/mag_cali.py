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
    
    def DrawData(self, X, Y, Z, Ainv):
        fig = plt.figure(figsize=(10, 10))
        ax = fig.add_subplot(111, projection='3d')
        ax.set_xlim(min(X), max(X))
        ax.set_ylim(min(Y), max(Y))
        ax.set_zlim(min(Z), max(Z))
        ax.scatter(X, Y, Z, marker='o', color='g')
        self.EllipsoidPlot(Ainv, ax)
        plt.show()
        
    # def EllipsoidPlot(self, center, radii, rotation, ax, plot_axes=False, cage_color='b', cage_alpha=0.2):
    def EllipsoidPlot(self, Ainv, ax, plot_axes=False, cage_color='b', cage_alpha=0.2):
        """Plot an ellipsoid"""
        
        # make some purdy axes
        axes = np.array([[1.0,0.0,0.0],
                         [0.0,1.0,0.0],
                         [0.0,0.0,1.0]])
        # rotate accordingly
        for i in range(len(axes)):
            axes[i] = np.dot(Ainv, axes[i])

        # plot axes
        for p in axes:
            X3 = np.linspace(-p[0], p[0], 100)
            Y3 = np.linspace(-p[1], p[1], 100)
            Z3 = np.linspace(-p[2], p[2], 100)
            ax.plot(X3, Y3, Z3, color=cage_color)
            
        # u = np.linspace(0.0, 2.0 * np.pi, 100)
        # v = np.linspace(0.0, np.pi, 100)
    
        # # cartesian coordinates that correspond to the spherical angles:
        # x = np.outer(np.cos(u), np.sin(v))
        # y = np.outer(np.sin(u), np.sin(v))
        # z = np.outer(np.ones_like(u), np.cos(v))
        # # rotate accordingly
        # for i in range(len(x)):
        #     for j in range(len(x)):
        #         [x[i, j], y[i, j], z[i, j]] = np.dot([x[i, j], y[i, j], z[i, j]], Ainv)

        # if plot_axes:
        #     # make some purdy axes
        #     axes = np.array([[radii[0],0.0,0.0],
        #                     [0.0,radii[1],0.0],
        #                     [0.0,0.0,radii[2]]])
        #     # rotate accordingly
        #     for i in range(len(axes)):
        #         axes[i] = np.dot(axes[i], rotation)

        #     # plot axes
        #     for p in axes:
        #         X3 = np.linspace(-p[0], p[0], 100) + center[0]
        #         Y3 = np.linspace(-p[1], p[1], 100) + center[1]
        #         Z3 = np.linspace(-p[2], p[2], 100) + center[2]
        #         ax.plot(X3, Y3, Z3, color=cage_color)

        # plot ellipsoid
        # ax.plot_wireframe(x, y, z,  rstride=4, cstride=4, color=cage_color, alpha=cage_alpha)

    def Calibrate(self, path):
        data_mag = np.loadtxt(self.project_root + path, dtype=np.float32)
        mag_x = data_mag[:, 0]
        mag_y = data_mag[:, 1]
        mag_z = data_mag[:, 2]
        
        Q, n, d = self.ellipsoid_fitting(mag_x, mag_y, mag_z)
        
        '''
        [1] Renaudin, Valerie, Muhammad Haris Afzal, and Gerard Lachapelle. "Complete Triaxis Magnetometer Calibration in the Magnetic Domain." Journal of Sensors (2010): 1-10.
        [2] Kok, Manon, et al. "Calibration of a magnetometer in combination with inertial sensors." international conference on information fusion (2012): 787-793.
        '''
        
        Qinv = np.linalg.inv(Q)
        # [2] Eq. 23b
        b = - 0.5 * np.dot(Qinv, n)
        # [2] Eq. 22
        alpha = 1.0 / (0.25 * np.dot(n.T, np.dot(Qinv, n)) - d)
        # [2] Eq. 23a
        Ainv = np.real(np.sqrt(alpha) * linalg.sqrtm(Q))
        
        num = data_mag.shape[0]
        bm = np.repeat(b, num, axis=1)
        data_mag_cali = np.dot(Ainv, data_mag.T - bm)
        # print(data_mag_cali)
        
        self.DrawData(data_mag_cali[0, :].tolist(), data_mag_cali[1, :].tolist(), data_mag_cali[2, :].tolist(), Ainv)


    def ellipsoid_fitting(self, X, Y, Z):
        ''' Estimate ellipsoid parameters from a set of points.

            Parameters
            ----------
            s : array_like
              The samples (M,N) where M=3 (x,y,z) and N=number of samples.

            Returns
            -------
            M, n, d : array_like, array_like, float
              The ellipsoid parameters M, n, d.

            References
            ----------
            .. [1] Qingde Li; Griffiths, J.G., "Least squares ellipsoid specific
               fitting," in Geometric Modeling and Processing, 2004.
               Proceedings, vol., no., pp.335-340, 2004
        '''

        # D (samples)
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

        # S, S_11, S_12, S_21, S_22 (eq. 11)
        S = np.dot(D, D.T)
        S_11 = S[:6,:6]
        S_12 = S[:6,6:]
        S_21 = S[6:,:6]
        S_22 = S[6:,6:]

        # C (Eq. 8, k=4)
        C = np.array([[-1,  1,  1,  0,  0,  0],
                      [ 1, -1,  1,  0,  0,  0],
                      [ 1,  1, -1,  0,  0,  0],
                      [ 0,  0,  0, -4,  0,  0],
                      [ 0,  0,  0,  0, -4,  0],
                      [ 0,  0,  0,  0,  0, -4]])

        # v_1 (eq. 15, solution)
        E = np.dot(linalg.inv(C),
                   S_11 - np.dot(S_12, np.dot(linalg.inv(S_22), S_21)))

        E_w, E_v = np.linalg.eig(E)

        v_1 = E_v[:, np.argmax(E_w)]
        if v_1[0] < 0: v_1 = -v_1

        # v_2 (eq. 13, solution)
        v_2 = np.dot(np.dot(-np.linalg.inv(S_22), S_21), v_1)

        # quadric-form parameters
        M = np.array([[v_1[0], v_1[3], v_1[4]],
                      [v_1[3], v_1[1], v_1[5]],
                      [v_1[4], v_1[5], v_1[2]]])
        n = 2 * np.array([[v_2[0]],
                      [v_2[1]],
                      [v_2[2]]])
        d = v_2[3]

        return M, n, d


if __name__ == "__main__":
    mag_cali = MagnetometerCalibration()
    # mag_cali.RecordMagData()
    # mag_cali.ShowMagData("/data/drone_mag_1590423949.txt")
    mag_cali.Calibrate("/data/drone_mag_1590423949.txt")
