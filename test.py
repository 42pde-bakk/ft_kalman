import numpy as np

np.set_printoptions(linewidth=100)

I = np.array([
    [1, 0, 0,  0,  0,  0],
    [0, 1, 0,  0,  0,  0],
    [0, 0, 1,  0,  0,  0],
    [0, 0, 0,  1,  0,  0],
    [0, 0, 0,  0,  1,  0],
    [0, 0, 0,  0,  0,  1],    
])

R = np.array([
    [9, 0, 0, 0],
    [0, 9, 0, 0],
    [0, 0, 9, 0],
    [0, 0, 0, 9],
])

P = np.array([
    [1125, 750, 250, 0, 0, 0],
    [750, 1000, 500, 0, 0, 0],
    [250,  500, 500, 0, 0, 0],
    [0, 0, 0, 1125, 750, 250],
    [0, 0, 0, 750, 1000, 250],
    [0, 0, 0, 250,  500, 500],
])

H = np.array([
    [0, 1, 0,  0,  0,  0],
    [0, 0, 1,  0,  0,  0],
    [0, 0, 0,  0,  1,  0],
    [0, 0, 0,  0,  0,  1],
])

# H = np.array([
#     [1, 0, 0, 0, 0, 0],
#     [0, 0, 0, 1, 0, 0],
# ])

# R = np.array([
#     [9, 0],
#     [0, 9],
# ])

K = P @ H.transpose() @ np.linalg.matrix_power(H @ P @ H.transpose(), -1)

print(K)

P = (I - K @ H) @ P @ (I - K @ H).transpose() + K @ R @ K.transpose()

print(P)