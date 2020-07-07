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
        
        return np.array(result), R

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

        x = np.zeros((4, 1))
        J = np.ones((num, 4))
        f = np.zeros((num, 1)) 

        px = np.zeros(num)
        py = np.zeros(num)
        pz = np.zeros(num)

        idx = 0
        while True:
            d = x[3, 0]
            R = SO3.exp(np.array([x[0, 0], x[1, 0], x[2, 0]])).mat

            for i in range(num):
                a = acc[i].reshape(3, 1)
                m = mag[i].reshape(3, 1)
                
                f[i] = d - np.dot(np.dot(a.T, R), m)
                
                ahat = np.array([[0, -a[2,0], a[1,0]], [a[2,0], 0, -a[0,0]], [-a[1,0], a[0, 0], 0]])

                J[i, :3] = - np.dot(np.dot(m.T, R), ahat)

            Jt_J_inv = np.linalg.inv(np.dot(J.T, J))
            dx = - np.dot(Jt_J_inv, np.dot(J.T, f))      
            x += dx
            
            dx_norm = np.linalg.norm(dx)
            print(str(idx) + ", " + str(dx_norm))
            idx+=1
            
            if(dx_norm < 1e-8):
                break
        
        d = x[3, 0]
        R = SO3.exp(np.array([x[0, 0], x[1, 0], x[2, 0]])).mat
        # print("======= Result =======")
        # print("R:")
        # print(R)
        # print("d:")
        # print(d)
        
        fig = plt.figure(figsize=(10, 10))
        ax = fig.add_subplot(111, projection='3d')
        ax.set_xlim(-1.2, 1.2)
        ax.set_ylim(-1.2, 1.2)
        ax.set_zlim(-1.2, 1.2)
        ax.set_xlabel('X axis')
        ax.set_ylabel('Y axis')
        ax.set_zlabel('Z axis')
        
        pu = acc[:, 0]
        pv = acc[:, 1]
        pw = acc[:, 2]

        ax.quiver(px, py, pz, pu, pv, pw, arrow_length_ratio=0.1, colors="red")

        # pu = mag[:, 0]
        # pv = mag[:, 1]
        # pw = mag[:, 2]

        # ax.quiver(px, py, pz, pu, pv, pw, arrow_length_ratio=0.1, colors="blue")

        m_align = (np.dot(R, mag.T)).T

        pu = m_align[:, 0]
        pv = m_align[:, 1]
        pw = m_align[:, 2]

        ax.plot(pu, pv, pw, 'ob', markersize=3)
        # ax.quiver(px, py, pz, pu, pv, pw, arrow_length_ratio=0.1, colors="yellow")

        plt.show()
        return R, d


if __name__ == "__main__":
    axes_align = AxesAlign()

    # theta = 0.1
    # R = np.array([[1, 0, 0], [0, np.cos(theta), -np.sin(theta)], [0, np.sin(theta), np.cos(theta)]])
    # print(R)
    # data = axes_align.TestDataGenerator(100, R, noise=False)

    data, R = axes_align.TestDataGenerator(1000, noise=False)
    
    acc = data[:, :3]
    mag = data[:, 3:]

    # axes_align.DrawData(acc, mag)

    R_fit, d_fit = axes_align.Align(acc, mag)
    
    print(R)
    print(R_fit)
    print(np.dot(R.T, R_fit))
    # print(np.linalg.norm(R_fit[:, 0]))
    # print(np.linalg.norm(R_fit[:, 1]))
    # print(np.linalg.norm(R_fit[:, 2]))
    
    


    # data = np.loadtxt(axes_align.project_root + "/data/drone_mag_1590423949.txt", dtype=np.float32)
    
    # mag_cali.Calibrate(mag_x, mag_y, mag_z)
    
