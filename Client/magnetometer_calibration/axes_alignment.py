import calendar
import atexit
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d import proj3d
from matplotlib.patches import FancyArrowPatch
import numpy as np
import sys
import os
import cv2
import math
from liegroups.numpy import SO3
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(
    os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))


class Arrow3D(FancyArrowPatch):
    def __init__(self, xs, ys, zs, *args, **kwargs):
        FancyArrowPatch.__init__(self, (0,0), (0,0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def draw(self, renderer):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, renderer.M)
        self.set_positions((xs[0],ys[0]),(xs[1],ys[1]))
        FancyArrowPatch.draw(self, renderer)


class AxesAlign:
    def __init__(self):
        self.project_root = os.path.dirname(os.path.realpath(__file__))

    def VirtualDataGenerator(self, num, theta=None, R=None, noise=False):
        if R is None:
            axis = np.random.standard_normal((3,))
            axis /= np.linalg.norm(axis)
            theta = np.random.uniform(0, 2*np.pi)
            axis *= theta
            R, _ = cv2.Rodrigues(axis)
        
        if theta is None:
            theta = np.random.uniform(low=-np.pi, high=np.pi)

        d = math.cos(theta)
        
        data_acc = np.zeros((3, num))
        data_mag = np.zeros((3, num))
    
        for i in range(num):
            mag = np.random.standard_normal((3))
            mag /= np.linalg.norm(mag)
            
            mag_n = np.dot(R, mag)
                        
            if noise is True:
                e = np.random.normal(0, 0.01, size=(3))
                theta += e
            
            while True:
                acc = np.random.standard_normal((3))
                
                A = mag_n[0] * acc[0] + mag_n[1] * acc[1]
                B = (acc[0] * acc[0] + acc[1] * acc[1]) * d * d
                D = mag_n[2] * mag_n[2] - d * d
                E = 2 * A * mag_n[2]
                F = A * A - B
                
                if (E * E - 4 * D * F) < 0:
                    continue
                
                acc[2] = (- E + math.sqrt(E * E - 4 * D * F)) / (2 * D)
                acc /= np.linalg.norm(acc)
                
                acc2 = -acc;
                
                e1 = abs(np.dot(acc.T, mag_n) - d)
                e2 = abs(np.dot(acc2.T, mag_n) - d)
                
                if(e2 < e1):
                    acc = acc2
                    
                break
            # acc = np.dot(R, mag)
            # if noise is True:
            #     e = np.random.normal(0, 0.01, size=(3))
            #     acc += e

            # acc /= np.linalg.norm(acc)

            data_acc[:, i] = acc
            data_mag[:, i] = mag
        
        return data_acc, data_mag, R, d
    
    def DrawAxes(self, R):
        fig = plt.figure(figsize=(10, 10))
        ax = fig.add_subplot(111, projection='3d')
        ax.set_xlim(-1.2, 1.2)
        ax.set_ylim(-1.2, 1.2)
        ax.set_zlim(-1.2, 1.2)
        ax.set_xlabel('X axis')
        ax.set_ylabel('Y axis')
        ax.set_zlabel('Z axis')
        
        acc_X = Arrow3D([0,1],[0,0],[0,0], mutation_scale=20, lw=1, arrowstyle="-|>", color="r")
        acc_Y = Arrow3D([0,0],[0,1],[0,0], mutation_scale=20, lw=1, arrowstyle="-|>", color="g")
        acc_Z = Arrow3D([0,0],[0,0],[0,1], mutation_scale=20, lw=1, arrowstyle="-|>", color="b")
        
        mag_X = Arrow3D([0, R[0, 0]],[0, R[0, 1]],[0, R[0, 2]], mutation_scale=20, lw=1, arrowstyle="-|>", color="r", linestyle="dashed")
        mag_Y = Arrow3D([0, R[1, 0]],[0, R[1, 1]],[0, R[1, 2]], mutation_scale=20, lw=1, arrowstyle="-|>", color="g", linestyle="dashed")
        mag_Z = Arrow3D([0, R[2, 0]],[0, R[2, 1]],[0, R[2, 2]], mutation_scale=20, lw=1, arrowstyle="-|>", color="b", linestyle="dashed")

        ax.add_artist(acc_X)
        ax.add_artist(acc_Y)
        ax.add_artist(acc_Z)
        ax.add_artist(mag_X)
        ax.add_artist(mag_Y)
        ax.add_artist(mag_Z)
    
        ax.scatter([0],[0],[0],color="y",s=20)

        plt.show()
        

    def DrawData(self, acc, mag, R=None, show_mag=False):
        num = acc.shape[1]

        fig = plt.figure(figsize=(10, 10))
        ax = fig.add_subplot(111, projection='3d')
        ax.set_xlim(-1.2, 1.2)
        ax.set_ylim(-1.2, 1.2)
        ax.set_zlim(-1.2, 1.2)
        ax.set_xlabel('X axis')
        ax.set_ylabel('Y axis')
        ax.set_zlabel('Z axis')
        
        px = np.zeros(num)
        py = np.zeros(num)
        pz = np.zeros(num)
        
        pu = acc[0, :]
        pv = acc[1, :]
        pw = acc[2, :]

        l1 = ax.quiver(px, py, pz, pu, pv, pw, arrow_length_ratio=0.1, linewidths=0.5, colors="red")

        if show_mag is True:
            pu = mag[0, :]
            pv = mag[1, :]
            pw = mag[2, :]
            ax.quiver(px, py, pz, pu, pv, pw, arrow_length_ratio=0.1, linewidths=0.5, colors="blue")

        if R is not None:
            m_align = np.dot(R, mag)

            pu = m_align[0, :]
            pv = m_align[1, :]
            pw = m_align[2, :]

            l2 = ax.plot(pu, pv, pw, 'ob', markersize=3)
            
            plt.legend(handles=[l1, l2[0]], labels=['Accelerometer vectors', 'Aligned Magnetometer vectors'])

        plt.show()

    def AlignRansac(self, acc, mag, max_itor=100, norm=False, debug_show=False):
        num = acc.shape[1]
        num_indexs = np.arange(num)

        if norm is True:
            acc /= np.linalg.norm(acc, axis=0)
            mag /= np.linalg.norm(mag, axis=0)

        idx = 0
        errors_best = 1e10
        R_best = np.eye(3)
        d_best = 0.0
        flag_best = False
        while True:
            indexs = np.random.choice(num_indexs, 500, replace=False)
            acc_sub = acc[:, indexs]
            mag_sub = mag[:, indexs]

            flag, R, d = self.Align(acc_sub, mag_sub, 30, norm=norm, debug_show=debug_show)

            if flag is True:
                errors = 0.0
                for i in range(num):
                    e = d - np.dot(acc[:, i].T, np.dot(R, mag[:, i]))
                    errors += (e*e)
                if errors_best > errors:
                    errors_best = errors
                    R_best = R.copy()
                    d_best = d
                    print(errors_best)


            flag_best = flag_best | flag

            idx += 1
            if idx > max_itor:
                break

        return flag_best, R_best, d_best


    def Align(self, acc, mag, max_itor=100, norm=False, debug_show=False):
        num = acc.shape[1]

        x = np.zeros((4, 1))
        J = np.ones((num, 4))
        f = np.zeros((num, 1)) 
        
        if norm is True:
            acc /= np.linalg.norm(acc, axis=0)
            mag /= np.linalg.norm(mag, axis=0)

        idx = 0
        flag = False
        while True:
            d = x[3, 0]
            R = SO3.exp(x[:3, 0]).mat

            for i in range(num):
                a = acc[:, i]
                m = mag[:, i]
                
                f[i] = d - np.dot(np.dot(a.T, R), m)
                
                ahat = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])

                J[i, :3] = - np.dot((np.dot(R, m)).T, ahat)

            Jt_J_inv = np.linalg.inv(np.dot(J.T, J))
            dx = - np.dot(Jt_J_inv, np.dot(J.T, f))      
            x += dx
            
            dx_norm = np.linalg.norm(dx)
            
            # if debug_show is True:
            #     print(str(idx) + ", " + str(dx_norm))
            
            idx+=1
            
            if dx_norm < 1e-8:
                flag = True
                break
            
            if idx > max_itor:
                break
        
        if flag is False:
            return False, np.eye(3), 0

        d = x[3, 0]
        R = SO3.exp(x[:3, 0]).mat
        
        if debug_show is True:
            # self.DrawData(acc, mag, R)
            self.DrawAxes(R)
        
        return True, R, d

    def TestVirtualData(self, num):
        # Method 1
        theta = np.pi / 4
        R = np.array([[1, 0, 0], [0, np.cos(theta), -np.sin(theta)], [0, np.sin(theta), np.cos(theta)]])
        data_acc, data_mag, R_gt, d_gt = self.VirtualDataGenerator(num, R, noise=False)
        
        # Method 2
        # data_acc, data_mag, R_gt = self.TestDataGenerator(num, noise=False)
        
        flag, R, d = self.Align(data_acc, data_mag, debug_show=True)
    
        print("R_gt")
        print(R_gt)
        print("d_gt")
        print(d_gt)
        print("R result")
        print(R)
        print("d")
        print(d)
    
    def TestRealData(self, path):
        data = np.loadtxt(self.project_root + path, dtype=np.float32, delimiter=",")

        acc = data[:, 1:4].T
        mag = data[:, 7:].T

        
        # flag, R, d = self.Align(acc, mag, norm=True, debug_show=True)

        flag, R, d = self.AlignRansac(acc, mag, 3000, norm=True, debug_show=False)
        
        print("R")
        print(R)
        print("d")
        print(d)

        # 17.929908083938276
        # R
        # [[ 9.98554420e-01  1.59317702e-02 -5.13346769e-02]
        #  [-1.59614459e-02  9.99872594e-01 -1.68150874e-04]
        #  [ 5.13254576e-02  9.87283469e-04  9.98681492e-01]]
        # d
        # 0.6394624321452079
        

if __name__ == "__main__":
    axes_align = AxesAlign()
    
    # axes_align.TestVirtualData(100)
    
    axes_align.TestRealData("/data/drone_imu_1594140390.txt")
    
