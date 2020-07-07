import calendar
import atexit
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import sys
import os
import cv2
from liegroups.numpy import SO3
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(
    os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))


class AxesAlign:
    def __init__(self):
        self.project_root = os.path.dirname(os.path.realpath(__file__))

    def TestDataGenerator(self, num, R=None, noise=False):
        if R is None:
            axis = np.random.standard_normal((3,))
            # print(axis)
            # print(np.linalg.norm(axis))
            axis /= np.linalg.norm(axis)
            # print(axis)
            theta = np.random.uniform(0, 2*np.pi)
            axis *= theta
            # print(axis)
            R, _ = cv2.Rodrigues(axis)

        result = []
        for i in range(num):
            acc = np.random.standard_normal((3, 1))
            acc /= np.linalg.norm(acc)

            mag = np.dot(R, acc)
            if noise is True:
                e = np.random.normal(0, 0.01, size=(3, 1))
                mag += e

            mag /= np.linalg.norm(mag)

            result.append([acc[0, 0], acc[1, 0], acc[2, 0], mag[0, 0], mag[1, 0], mag[2, 0]])
        
        return np.array(result)

    def DrawData(self, acc, mag):
        num = acc.shape[0]

        fig = plt.figure(figsize=(10, 10))
        ax = fig.add_subplot(111, projection='3d')
        ax.set_xlim(-1.2, 1.2)
        ax.set_ylim(-1.2, 1.2)
        ax.set_zlim(-1.2, 1.2)
        ax.set_xlabel('X axis')
        ax.set_ylabel('Y axis')
        ax.set_zlabel('Z axis')

        x = np.zeros(num)
        y = np.zeros(num)
        z = np.zeros(num)
        u = acc[:, 0]
        v = acc[:, 1]
        w = acc[:, 2]

        ax.quiver(x, y, z, u, v, w, arrow_length_ratio=0.1, colors="red")

        u = mag[:, 0]
        v = mag[:, 1]
        w = mag[:, 2]

        ax.quiver(x, y, z, u, v, w, arrow_length_ratio=0.1, colors="blue")

        plt.show()

    def Align(self, acc, mag):
        num = acc.shape[0]

        x = np.zeros(4)
        J = np.zeros((num, 4))

        px = np.zeros(num)
        py = np.zeros(num)
        pz = np.zeros(num)

        while True:
            d = x[3]
            R = SO3.exp(np.array([x[0], x[1], x[2]])).mat

            pu = acc[:, 0]
            pv = acc[:, 1]
            pw = acc[:, 2]

            
            fig = plt.figure(figsize=(10, 10))
            ax = fig.add_subplot(111, projection='3d')
            ax.set_xlim(-1.2, 1.2)
            ax.set_ylim(-1.2, 1.2)
            ax.set_zlim(-1.2, 1.2)
            ax.set_xlabel('X axis')
            ax.set_ylabel('Y axis')
            ax.set_zlabel('Z axis')




            ax.quiver(px, py, pz, pu, pv, pw, arrow_length_ratio=0.1, colors="red")

            pu = mag[:, 0]
            pv = mag[:, 1]
            pw = mag[:, 2]

            ax.quiver(px, py, pz, pu, pv, pw, arrow_length_ratio=0.1, colors="blue")

            m_align = (np.dot(R, mag.T)).T

            pu = m_align[:, 0]
            pv = m_align[:, 1]
            pw = m_align[:, 2]

            ax.quiver(px, py, pz, pu, pv, pw, arrow_length_ratio=0.1, colors="yellow")

            plt.show()

            for i in range(num):
                a = acc[i].reshape(3, 1)
                m = mag[i].reshape(3, 1)

                J[i, 3] = - np.dot(np.dot(a.T, R), m)
                
                ahat = np.array([[0, -a[2,0], a[1,0]], [a[2,0], 0, -a[0,0]], [-a[1,0], a[0, 0], 0]])

                J[i, :3] = - np.dot(np.dot(m.T, R), ahat)
            
            f = J[:, 3] + d

            Jt_J_inv = np.linalg.inv(np.dot(J.T, J))

            dx = - np.dot(Jt_J_inv, np.dot(J.T, f))

            x += dx

            print(dx)
            print(x)
            # input("wait\n")
            # os.system("pause")
            # print(R)
            # exit(1)


        # print(x_init)


if __name__ == "__main__":
    axes_align = AxesAlign()

    theta = 0.1
    R = np.array([[1, 0, 0], [0, np.cos(theta), -np.sin(theta)], [0, np.sin(theta), np.cos(theta)]])
    data = axes_align.TestDataGenerator(100, R, noise=False)

    # data = axes_align.TestDataGenerator(10, noise=False)
    
    acc = data[:, :3]
    mag = data[:, 3:]

    # axes_align.DrawData(acc, mag)

    axes_align.Align(acc, mag)


    # data = np.loadtxt(axes_align.project_root + "/data/drone_mag_1590423949.txt", dtype=np.float32)
    
    # mag_cali.Calibrate(mag_x, mag_y, mag_z)
    
