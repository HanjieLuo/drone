import queue
import mavlink_task
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from PyQt5 import QtCore, QtWidgets
import sys
import random
import matplotlib
matplotlib.use('Qt5Agg')


class MplCanvas(FigureCanvas):

    def __init__(self, parent=None, width=5, height=4, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = list()
        self.axes.append(fig.add_subplot(311))
        self.axes.append(fig.add_subplot(312))
        self.axes.append(fig.add_subplot(313))
        fig.tight_layout(pad=1.5)
        super(MplCanvas, self).__init__(fig)


class MainWindow(QtWidgets.QMainWindow):

    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)

        self.canvas = MplCanvas(self, width=10, height=8, dpi=100)
        self.setCentralWidget(self.canvas)

        self.mavlink_task = mavlink_task.MavkinkTask("COM5", 115200)
        self.mavlink_task.start()

        self.n_data = 500
        self.delay = 10 #ms
        self.basetime = None
        self.xdata = list()
        self.xacc_data = list()
        self.yacc_data = list()
        self.zacc_data = list()
        self.xgyro_data = list()
        self.ygyro_data = list()
        self.zgyro_data = list()
        self.xmag_data = list()
        self.ymag_data = list()
        self.zmag_data = list()
        # self.ydata = [random.randint(0, 10) for i in range(n_data)]

        # We need to store a reference to the plotted line
        # somewhere, so we can apply the new data to it.
        self._plot_ref = None
        self.update_plot()

        self.show()

        # Setup a timer to trigger the redraw by calling update_plot.
        self.timer = QtCore.QTimer()
        self.timer.setInterval(1)
        self.timer.timeout.connect(self.update_plot)
        self.timer.start()

    def update_plot(self):
        try:
            [timestamp, xacc, yacc, zacc, xgyro, ygyro, zgyro, xmag,
                ymag, zmag] = self.mavlink_task.imu_data.get(block=False)
            print(self.mavlink_task.imu_data.qsize())
            # print(self.mavlink_task.imu_data.qsize())
            # print('%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f' %
            #               (timestamp, xacc, yacc, zacc, xgyro, ygyro, zgyro, xmag, ymag, zmag))
            if self.basetime is None:
                self.basetime = timestamp

            if(len(self.xgyro_data) < self.n_data - 1):
                self.xdata.append(timestamp - self.basetime)

                self.xacc_data.append(xacc)
                self.yacc_data.append(yacc)
                self.zacc_data.append(zacc)

                self.xgyro_data.append(xgyro)
                self.ygyro_data.append(ygyro)
                self.zgyro_data.append(zgyro)

                self.xmag_data.append(xmag)
                self.ymag_data.append(ymag)
                self.zmag_data.append(zmag)
            else:
                self.xdata = self.xdata[1:] + [timestamp - self.basetime]

                self.xacc_data = self.xacc_data[1:] + [xacc]
                self.yacc_data = self.yacc_data[1:] + [yacc]
                self.zacc_data = self.zacc_data[1:] + [zacc]

                self.xgyro_data = self.xgyro_data[1:] + [xgyro]
                self.ygyro_data = self.ygyro_data[1:] + [ygyro]
                self.zgyro_data = self.zgyro_data[1:] + [zgyro]

                self.xmag_data = self.xmag_data[1:] + [xmag]
                self.ymag_data = self.ymag_data[1:] + [ymag]
                self.zmag_data = self.zmag_data[1:] + [zmag]

                xlim1, xlim2 = self.canvas.axes[0].get_xlim()
                for i in range(3):
                    self.canvas.axes[i].set_xlim(xlim1 + self.delay, xlim2 + self.delay)

            # Note: we no longer need to clear the axis.
            if self._plot_ref is None:
                # First time we have no plot reference, so do a normal plot.
                # .plot returns a list of line <reference>s, as we're
                # only getting one we can take the first element.
                self._plot_ref = list()
                self._plot_ref.append(self.canvas.axes[0].plot(
                    self.xdata, self.xacc_data, 'r', self.xdata, self.yacc_data, 'g', self.xdata, self.zacc_data, 'b'))
                
                self._plot_ref.append(self.canvas.axes[1].plot(
                    self.xdata, self.xgyro_data, 'r',  self.xdata, self.ygyro_data, 'g', self.xdata, self.zgyro_data, 'b'))
                
                self._plot_ref.append(self.canvas.axes[2].plot(
                    self.xdata, self.xmag_data, 'r',  self.xdata, self.ymag_data, 'g', self.xdata, self.zmag_data, 'b'))

                for i in range(3):
                    self.canvas.axes[i].set_xlim(0, self.delay * self.n_data)
                    self._plot_ref[i][0].set_label("X Axis")
                    self._plot_ref[i][1].set_label("Y Axis")
                    self._plot_ref[i][2].set_label("Z Axis")
                    self.canvas.axes[i].legend()
                
                self.canvas.axes[0].set_ylim(-20, 20)
                self.canvas.axes[1].set_ylim(-35, 35)
                self.canvas.axes[2].set_ylim(-1, 1)
                
                self.canvas.axes[0].set_title("Acceleration")
                self.canvas.axes[1].set_title("Gyroscope")
                self.canvas.axes[2].set_title("Magnetometer")
            else:
                # We have a reference, we can use it to update the data for that line.
                self._plot_ref[0][0].set_xdata(self.xdata)
                self._plot_ref[0][0].set_ydata(self.xacc_data)
                self._plot_ref[0][1].set_xdata(self.xdata)
                self._plot_ref[0][1].set_ydata(self.yacc_data)
                self._plot_ref[0][2].set_xdata(self.xdata)
                self._plot_ref[0][2].set_ydata(self.zacc_data)
                
                self._plot_ref[1][0].set_xdata(self.xdata)
                self._plot_ref[1][0].set_ydata(self.xgyro_data)
                self._plot_ref[1][1].set_xdata(self.xdata)
                self._plot_ref[1][1].set_ydata(self.ygyro_data)
                self._plot_ref[1][2].set_xdata(self.xdata)
                self._plot_ref[1][2].set_ydata(self.zgyro_data)
                
                self._plot_ref[2][0].set_xdata(self.xdata)
                self._plot_ref[2][0].set_ydata(self.xmag_data)
                self._plot_ref[2][1].set_xdata(self.xdata)
                self._plot_ref[2][1].set_ydata(self.ymag_data)
                self._plot_ref[2][2].set_xdata(self.xdata)
                self._plot_ref[2][2].set_ydata(self.zmag_data)
            # Trigger the canvas to update and redraw.
            self.canvas.draw()
        except queue.Empty:
            pass
        except Exception as e:
            print("[Error] IMU CALI APP: ", str(e))
        # # Drop off the first y element, append a new one.
        # self.ydata = self.ydata[1:] + [random.randint(0, 10)]


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    w = MainWindow()
    sys.exit(app.exec_())
