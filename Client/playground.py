import numpy as np


# // const static float Ta00 = 1.0, Ta01 = 9.0941e-05, Ta02 = -0.00458356;
# // const static float Ta10 = 0.0, Ta11 = 1.0, Ta12 = -0.00704908;
# // const static float Ta20 = 0.0, Ta21 = 0.0, Ta22 = 1.0;
# // const static float Sa_x = -0.000596561, Sa_y = -0.000594778, Sa_z = -0.000594165;

Ta = np.array([[1., 9.0941e-05, -0.00458356],
               [0., 1.,          -0.00704908],
               [0., 0, 1.0]])

Sa = np.diag([-0.000596561, -0.000594778, -0.000594165])


np.set_printoptions(precision=32)
print(Ta)
print(Sa)
print(Ta.dot(Sa))

Tg = np.array([[1., -0.0006364, 0.00508642],
               [-0.00272323, 1.0, 0.00577732],
               [0.00824933, 0.012745, 1.0]])

Sg = np.diag([0.00106156, 0.00105634, 0.00107057])

print(Tg)
print(Sg)
print(Tg.dot(Sg))