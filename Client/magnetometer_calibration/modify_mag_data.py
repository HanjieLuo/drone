import numpy as np

data_mag = np.loadtxt("C:/Users/luoha/Documents/workspace/drone/code/Client/magnetometer_calibration/data/drone_mag_1590423949.txt", dtype=np.float32)

tmp = np.copy(data_mag[:, 2])
data_mag[:, 2] = data_mag[:, 1]
data_mag[:, 1] = tmp

mag_file = open("C:/Users/luoha/Documents/workspace/drone/code/Client/magnetometer_calibration/data/drone_mag_1590423949.txt", 'w')

rows = data_mag.shape[0]
for y in range(0, rows -1):
    mag_data = '%f %f %f\n' % (data_mag[y, 0], data_mag[y, 1], data_mag[y, 2])
    mag_file.write(mag_data)

mag_file.close()

# -142.000000 -67.000000 554.000000