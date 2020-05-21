# -*- coding: UTF-8 -*-
import numpy as np

data_acc = np.loadtxt("data/drone_acc_1590081763.txt", dtype=np.float32)
data_gyro = np.loadtxt("data/drone_gyro_1590081763.txt", dtype=np.float32)

print(data_acc.shape)
print(data_gyro.shape)

# ===================================================
# data_acc[:,1:] = data_acc[:,1:] / 0.000597125244140625
# data_gyro[:,1:] = data_gyro[:,1:] / 0.00106422515

# acc_file = open("data/drone_acc_1589735545_m.txt", 'w')
# gyro_file = open("data/drone_gyro_1589735545_m.txt", 'w')

# rows = data_acc.shape[0]

# for y in range(0, rows -1):
#     acc_data = '   %f   %f   %f   %f\n' % (data_acc[y, 0], data_acc[y, 1], data_acc[y, 2], data_acc[y, 3])
#     gyro_data = '   %f   %f   %f   %f\n' % (data_gyro[y, 0], data_gyro[y, 1], data_gyro[y, 2], data_gyro[y, 3])
#     acc_file.write(acc_data)
#     gyro_file.write(gyro_data)

# acc_file.close()
# gyro_file.close()      

# print(data_acc)
# print(data_gyro)
# print(data_acc[:,1:] * 0.000597125244140625)
# ===================================================
