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
    
    def RecordCaliData(self):
        # recored the calibrated acc, gryo and mag data, make sure make the IMU static at the first 50s
        from com_task import ComTask
        self.com_task = ComTask("COM5", 115200)
        self.com_task.start()

        ts = calendar.timegm(time.gmtime())
        mag_file = open(self.project_root +
                        '/data/drone_calied_acc_gyro_mag_' + str(ts) + '.txt', 'w')

        def cleanup():
            print("Exit")
            mag_file.close()

        atexit.register(cleanup)
                
        self.mag_x = list()
        self.mag_y = list()
        self.mag_z = list()
        self.plot = None
        
        fig = plt.figure(figsize=(10, 10))
        self.ax = fig.add_subplot(111, projection='3d')
        
        index = 0 
        while True:
            try:
                [timestamp, xacc, yacc, zacc, xgyro, ygyro, zgyro,
                    xmag, ymag, zmag] = self.com_task.getData()
            except:
                print("Error reading")
                continue
            
            data = '%d,%f,%f,%f,%f,%f,%f,%f,%f,%f\n' % (timestamp, xacc, yacc, zacc, xgyro, ygyro, zgyro, xmag, ymag, zmag)
            print(data)

            # mag_data = '%f %f %f\n' % (xmag, ymag, zmag)
            mag_file.write(data)
            # print(index)
            if index % 10 == 0: 
                # print("print")           
                self.mag_x.append(xmag)
                self.mag_y.append(ymag)
                self.mag_z.append(zmag)

                self.ax.cla()
                self.ax.set_xlim(-1, 1)
                self.ax.set_ylim(-1, 1)
                self.ax.set_zlim(-1, 1)
                self.plot = self.ax.scatter(
                    self.mag_x, self.mag_y, self.mag_z, marker='o', color='g')

            index += 1
            plt.pause(0.0001)  # 设置时间间隔
        plt.show()


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
        
        self.DrawAxes(R_best)

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

    def RunVirtualData(self, num):
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
    
    def StaticIntervalsDetector(self, data, T_init=50, T_wait=2, static_std_scale=2, debug_show=False):
        data_num = data.shape[0]

        acc = data[:, 1:4]
        mag = data[:, 7:]
        dt =  (data[:, 0] - data[0, 0] ) * 0.001
        
        idx_init_end = 0
        for i in range(data_num):
            if dt[i] > T_init:
                idx_init_end = i
                break
        
        acc_init = acc[:idx_init_end, :]
        acc_static_std = np.std(acc_init, axis=0, ddof=1) * static_std_scale
        # print(acc_static_std)
        
        win_size = np.round(T_wait / dt[1])
        if win_size % 2 == 0:
            win_size += 1
        half_win_size = int(win_size / 2)

        static_intervals = []
        static_indexs = []
        find_interval_start = False
        start_id = 0
        for i in range(idx_init_end + half_win_size, data_num - half_win_size):
            acc_interval = acc[i - half_win_size: i + half_win_size + 1, :]
            acc_interval_std = np.std(acc_interval, axis=0, ddof=1)
            if acc_interval_std[0] <= acc_static_std[0] and acc_interval_std[1] <= acc_static_std[1] and acc_interval_std[2] <= acc_static_std[2]:
                if find_interval_start is False:
                    start_id = i
                    find_interval_start = True
                static_indexs.append(i)
            else:
                if find_interval_start is True:
                    static_intervals.append([start_id, i - 1])
                    find_interval_start = False
        if find_interval_start is True:
            static_intervals.append([start_id, i])

        acc_static = acc.take(static_indexs, axis=0)
        mag_static = mag.take(static_indexs, axis=0)
            
        if debug_show is True:
            # print(len(static_intervals))
            static_show = []
            for idx in static_intervals:
                static_show.append((dt[idx[0]], dt[idx[1]] - dt[idx[0]]))
            
            # c = sys.stdin.read(1)
            # print(static_intervals)
            fig, ax = plt.subplots() 
            # plt.plot(dt, acc[:, 0], 'r-', dt, acc[:, 1], 'g-', dt, acc[:, 2], 'b-')
            ax.plot(dt, acc[:, 0], 'r-', label='acc x')  # Plot some data on the axes.
            ax.plot(dt, acc[:, 1], 'g-', label='acc y')  # Plot more data on the axes...
            ax.plot(dt, acc[:, 2], 'b-', label='acc z')  # ... and some more.
            ax.broken_barh(static_show, (-20, 40), facecolors ='tab:cyan', label='static interval') 
            ax.set_ylim(-20, 20)
            ax.legend()
            plt.show()
        
        return acc_static, mag_static
        
    def RunRealData(self, path):
        data = np.loadtxt(self.project_root + path, dtype=np.float32, delimiter=",")

        # remove oulider of mag
        mag_norm = np.linalg.norm(data[:, 7:], axis=1)
        data_inlider = data[(mag_norm<1.2) & (mag_norm > 0.8)]
        
        acc_static, mag_static = self.StaticIntervalsDetector(data_inlider)
        # x = np.arange(acc_static.shape[0])
        # plt.plot(x, acc_static[:, 0], 'r-', x, acc_static[:, 1], 'g-', x, acc_static[:, 2], 'b-')
        # plt.show()
        
        # flag, R, d = self.Align(acc_static.T, mag_static.T, norm=True, debug_show=True)

        flag, R, d = self.AlignRansac(acc_static.T, mag_static.T, 3000, norm=True, debug_show=False)
        
        
        np.set_printoptions(precision=10)
        print("R")
        print(R)
        print("d")
        print(d)

        # error: 6.020256221141558
        # R
        # [[ 0.9996613759 -0.0145143345  0.0215978633]
        #  [ 0.0144040375  0.9998824543  0.0052536868]
        #  [-0.0216715783 -0.0049408113  0.999752935 ]]
        # d
        # 0.6433718256321436
        # Tm = R * Ainv
        # [[ 2.2754987917e-03 -1.3506234138e-05  1.1826960352e-04]
        # [ 5.1905146781e-05  2.2232194724e-03  1.2689010139e-05]
        # [ 1.3161782481e-05 -1.3150356768e-05  2.5764247895e-03]]

        return R, d
        

if __name__ == "__main__":
    axes_align = AxesAlign()
    
    # axes_align.RunVirtualData(100)
    
    # axes_align.RecordCaliData()
    R, d = axes_align.RunRealData("/data/drone_calied_acc_gyro_mag_1612700585.txt")
    
    # R = np.array( [[0.9996613759, -0.0145143345,  0.0215978633],[0.0144040375,  0.9998824543,  0.0052536868],[-0.0216715783, -0.0049408113,  0.999752935]])
    # Ainv = np.array( [[ 2.27519066e-03,1.88066651e-05,6.25771359e-05],[ 1.88066651e-05,2.22321915e-03,-1.75871479e-06],[ 6.25771359e-05,-1.75871479e-06,2.57840928e-03]])
    # np.set_printoptions(precision=10)
    # print(np.dot(R,Ainv))
    