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
    
    def DrawCircle(self, X, Y, Z, ax=None):
        if ax is None:
            fig = plt.figure(figsize=(10, 10))
            ax = fig.add_subplot(111, projection='3d')
            fig.suptitle('Calibration Data', fontsize=20)
        value = X + Y + Z
        max_value = max([abs(x) for x in value]) * 1.2
        ax.set_xlim(-max_value, max_value)
        ax.set_ylim(-max_value, max_value)
        ax.set_zlim(-max_value, max_value)
        ax.scatter(X, Y, Z, marker='o', color='g')
        
    def DrawEllipsoid(self, X, Y, Z, A, b, title, cage_alpha=0.2):
        """Plot an ellipsoid"""
        
        fig = plt.figure(figsize=(10, 10))
        ax = fig.add_subplot(111, projection='3d')
        fig.suptitle(title, fontsize=20)
        
        value = X + Y + Z
        max_value = max([abs(x) for x in value]) * 1.2
        ax.set_xlim(-max_value, max_value)
        ax.set_ylim(-max_value, max_value)
        ax.set_zlim(-max_value, max_value)
        ax.set_xlabel('X axis')
        ax.set_ylabel('Y axis')
        ax.set_zlabel('Z axis')
        ax.scatter(X, Y, Z, s=1, marker='o', color='g')
        
        center = b.flatten()
        axes = np.array([[1.0, 0.0, 0.0],
                         [0.0, 1.0, 0.0],
                         [0.0, 0.0, 1.0]])
        for i in range(3):
            p1 = np.dot(A, axes[:, i]) + center
            p2 = np.dot(A, -axes[:, i]) + center
            X3 = np.linspace(p1[0], p2[0], 100)
            Y3 = np.linspace(p1[1], p2[1], 100)
            Z3 = np.linspace(p1[2], p2[2], 100)
            ax.plot(X3, Y3, Z3, color="orange")
             
        u = np.linspace(0.0, 2.0 * np.pi, 100)
        v = np.linspace(0.0, np.pi, 100)
    
        # cartesian coordinates that correspond to the spherical angles:
        x = np.outer(np.cos(u), np.sin(v))
        y = np.outer(np.sin(u), np.sin(v))
        z = np.outer(np.ones_like(u), np.cos(v))

        # rotate accordingly
        for i in range(len(x)):
            for j in range(len(x)):
                # print(np.dot(A, np.array([[x[i, j]], [y[i, j]], [z[i, j]]])) + b)
                # print(np.sqrt(x[i, j] ** 2 + y[i, j] ** 2 + z[i, j] ** 2))
                [x[i, j], y[i, j], z[i, j]] = np.dot(A, np.array([[x[i, j]], [y[i, j]], [z[i, j]]])) + b
    
        
        # print(np.dot(A, np.array([[1], [0], [0]])) + center)

        ax.plot_wireframe(x, y, z,  rstride=4, cstride=4, color="orange", alpha=cage_alpha)
    
    def DrawEllipsoidAxes(self, X, Y, Z, A, b, axes_len, axes_dir, title, cage_alpha=0.2):
        """Plot an ellipsoid"""
        
        fig = plt.figure(figsize=(10, 10))
        ax = fig.add_subplot(111, projection='3d')
        fig.suptitle(title, fontsize=20)
        
        
        value = X + Y + Z
        max_value = max([abs(x) for x in value]) * 1.2
        ax.set_xlim(-max_value, max_value)
        ax.set_ylim(-max_value, max_value)
        ax.set_zlim(-max_value, max_value)
        ax.set_xlabel('X axis')
        ax.set_ylabel('Y axis')
        ax.set_zlabel('Z axis')
        ax.scatter(X, Y, Z, s=1, marker='o', color='g')
        
        center = b.flatten()
        axes = np.array([[1.0, 0.0, 0.0],
                         [0.0, 1.0, 0.0],
                         [0.0, 0.0, 1.0]])
        for i in range(3):
            p1 = np.dot(A, axes[:, i]) + center
            p2 = np.dot(A, -axes[:, i]) + center
            X3 = np.linspace(p1[0], p2[0], 100)
            Y3 = np.linspace(p1[1], p2[1], 100)
            Z3 = np.linspace(p1[2], p2[2], 100)
            ax.plot(X3, Y3, Z3, color="orange")
                    
            p1 = axes_dir[:, i] * axes_len[i] + center
            p2 = -axes_dir[:, i] * axes_len[i] + center
            X3 = np.linspace(p1[0], p2[0], 100)
            Y3 = np.linspace(p1[1], p2[1], 100)
            Z3 = np.linspace(p1[2], p2[2], 100)
            ax.plot(X3, Y3, Z3, color="blue")
             
        u = np.linspace(0.0, 2.0 * np.pi, 100)
        v = np.linspace(0.0, np.pi, 100)
    
        # cartesian coordinates that correspond to the spherical angles:
        x = np.outer(np.cos(u), np.sin(v))
        y = np.outer(np.sin(u), np.sin(v))
        z = np.outer(np.ones_like(u), np.cos(v))

        # rotate accordingly
        for i in range(len(x)):
            for j in range(len(x)):
                [x[i, j], y[i, j], z[i, j]] = np.dot(A, np.array([[x[i, j]], [y[i, j]], [z[i, j]]])) + b

        ax.plot_wireframe(x, y, z,  rstride=4, cstride=4, color="orange", alpha=cage_alpha)

    def Calibrate(self, mag_x, mag_y, mag_z):        
        Q, n, d = self.EllipsoidFitting(mag_x, mag_y, mag_z)
        
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
        
        print("==================")
        print("Calibration Result")
        print("==================")
        print("b:")
        print(b)
        print("Ainv:")
        print(Ainv)
        print("cali_data = Ainv * (raw_data - b)")
                
        A = np.linalg.inv(Ainv)
        
        self.DrawEllipsoid(mag_x.tolist(), mag_y.tolist(), mag_z.tolist(), A, b, "Raw Data")
        self.DrawEllipsoid(data_mag_cali[0, :].tolist(), data_mag_cali[1, :].tolist(), data_mag_cali[2, :].tolist(), np.identity(3), np.array([[0], [0], [0]]), "Calibration Data")
        
        # # check EllipsoidFitting2
        # b, Ainv, axes_dir, axes_len = self.EllipsoidFitting2(mag_x, mag_y, mag_z)
        
        # # Compare two axes computation method
        Q = alpha * Q
        axes_len, axes_dir = np.linalg.eigh(Q)
        axes_len = 1.0 / np.sqrt(axes_len)
        self.DrawEllipsoidAxes(mag_x.tolist(), mag_y.tolist(), mag_z.tolist(), A, b, axes_len, axes_dir, "DrawEllipsoidAxes")
        
        plt.show()
    
    def GenerateEllipsoidPoints(self, a=0.6, b=0.3, c=0.3, debug=False):        
        twopi = 2.0 * np.pi

        # z = np.linspace(-1, 1, 90)[1:-1]
        z = np.linspace(-1, 1, 90)
        r = np.sqrt(1.0 - z**2)[:,None]
        theta = np.linspace(0, twopi, 90, endpoint=True).flatten()
        # theta = np.linspace(0, twopi, 89, endpoint=True)[:-1][None,:]
        x = r*np.cos(theta)
        y = r*np.sin(theta)

        data = []
        
        if debug is True:
            fig = plt.figure(figsize=(10, 10))
            ax = fig.add_subplot(111, projection='3d')
            fig.suptitle("GenerateEllipsoidPoints", fontsize=20)
        
        for i in range(90):
            if debug is True:
                ax.plot(a*x[:,i], b*y[:,i], c*z, 'ok', markersize=1)
            for j in range(90):
                data.append([a*x[j,i], b*y[j,i], c*z[j]])
        
        if debug is True:      
            ax.set_xlim(-1, 1)
            ax.set_ylim(-1, 1)
            ax.set_zlim(-1, 1)
            plt.show()
        
        return np.array(data)


    def EllipsoidFitting(self, X, Y, Z):
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
    
    def EllipsoidFitting2(self, x, y, z):
        # x = X[:, 0]
        # y = X[:, 1]
        # z = X[:, 2]
        D = np.array([x * x + y * y - 2 * z * z,
                    x * x + z * z - 2 * y * y,
                    2 * x * y,
                    2 * x * z,
                    2 * y * z,
                    2 * x,
                    2 * y,
                    2 * z,
                    1 - 0 * x])
        d2 = np.array(x * x + y * y + z * z).T # rhs for LLSQ
        u = np.linalg.solve(D.dot(D.T), D.dot(d2))
        a = np.array([u[0] + 1 * u[1] - 1])
        b = np.array([u[0] - 2 * u[1] - 1])
        c = np.array([u[1] - 2 * u[0] - 1])
        v = np.concatenate([a, b, c, u[2:]], axis=0).flatten()
        A = np.array([[v[0], v[3], v[4], v[6]],
                    [v[3], v[1], v[5], v[7]],
                    [v[4], v[5], v[2], v[8]],
                    [v[6], v[7], v[8], v[9]]])

        center = np.linalg.solve(- A[:3, :3], v[6:9])

        translation_matrix = np.eye(4)
        translation_matrix[3, :3] = center.T

        R = translation_matrix.dot(A).dot(translation_matrix.T)

        evals, evecs = np.linalg.eig(R[:3, :3] / -R[3, 3])
        evecs = evecs.T

        radii = np.sqrt(1. / np.abs(evals))
        radii *= np.sign(evals)
        offset = center

        a, b, c = radii
        D = np.array([[1/a, 0., 0.], [0., 1/b, 0.], [0., 0., 1/c]])
        transform = evecs.dot(D).dot(evecs.T)
        
        axes_dir = evecs
        axes_len = radii

        return np.reshape(offset, (3, 1)), transform, axes_dir, axes_len


if __name__ == "__main__":
    mag_cali = MagnetometerCalibration()

    # mag_cali.RecordMagData()
    # mag_cali.ShowMagData("/data/drone_mag_1590423949.txt")
        
    # data_mag = mag_cali.GenerateEllipsoidPoints()
    data_mag = np.loadtxt(mag_cali.project_root + "/data/drone_mag_1590423949.txt", dtype=np.float32)
    
    mag_x = data_mag[:, 0]
    mag_y = data_mag[:, 1]
    mag_z = data_mag[:, 2]
    mag_cali.Calibrate(mag_x, mag_y, mag_z)
    
