import math
from typing import List
import numpy as np
import pandas as pd


np.set_printoptions(linewidth=100, suppress=True)

# F = np.array([
#     [1, 1, 0.5,0,  0,  0],
#     [0, 1, 1,  0,  0,  0],
#     [0, 0, 1,  0,  0,  0],
#     [0, 0, 0,  1,  1,  0.5],
#     [0, 0, 0,  0,  1,  1],
#     [0, 0, 0,  0,  0,  1],    
# ])

def make_f_mat(delta):
    vel = delta * delta * 0.5
    F = np.array([
        [1, delta, vel,0,  0,  0],
        [0, 1, delta,  0,  0,  0],
        [0, 0, 1,  0,  0,  0],
        [0, 0, 0,  1,  delta,  vel],
        [0, 0, 0,  0,  1,  delta],
        [0, 0, 0,  0,  0,  1],    
    ])

    return F

def make_q_mat(delta):
    Q = np.array([
        [delta ** 4 / 4, delta ** 3 / 2, delta ** 2 / 2, 0, 0, 0],
        [delta ** 3 / 2, delta ** 2    , delta         , 0, 0, 0],
        [delta ** 2 / 2, delta         , 1             , 0, 0, 0],
        [0, 0, 0, delta ** 4 / 4, delta ** 3 / 2, delta ** 2 / 2],
        [0, 0, 0, delta ** 3 / 2, delta ** 2    , delta         ],
        [0, 0, 0, delta ** 2 / 2, delta         , 1             ],
    ])

    return Q

def calculate_velocity(direction: List[float], speed: int):
    pitch = direction[1]
    yaw = direction[2]

    return [
        speed * math.cos(pitch) * math.cos(yaw),
        speed * math.cos(pitch) * math.sin(yaw),
        speed * math.sin(pitch),
    ]


I = np.array([
    [1, 0, 0,  0,  0,  0],
    [0, 1, 0,  0,  0,  0],
    [0, 0, 1,  0,  0,  0],
    [0, 0, 0,  1,  0,  0],
    [0, 0, 0,  0,  1,  0],
    [0, 0, 0,  0,  0,  1],    
])

R = np.array([
    [0.04, 0, 0, 0],
    [0, 0.04, 0, 0],
    [0, 0, 0.04, 0],
    [0, 0, 0, 0.04],
])

P = np.array([
    [1125, 750, 250, 0, 0, 0],
    [750, 1000, 500, 0, 0, 0],
    [250,  500, 500, 0, 0, 0],
    [0, 0, 0, 1125, 750, 250],
    [0, 0, 0, 750, 1000, 500],
    [0, 0, 0, 250,  500, 500],
])

H = np.array([
    [0, 1, 0,  0,  0,  0],
    [0, 0, 1,  0,  0,  0],
    [0, 0, 0,  0,  1,  0],
    [0, 0, 0,  0,  0,  1],
])

Q = np.array([
    [1/4, 1/2, 1/2, 0,  0,  0],
    [1/2,  1, 1,    0,  0,  0],
    [1/2,  1, 1,    0,  0,  0],
    [0, 0, 0,   1/4, 1/2, 1/2],
    [0, 0, 0,   1/4,   1,   1],
    [0, 0, 0,   1/2,   1,   1],    
]) * (0.2 * 0.2)

# H = np.array([
#     [1, 0, 0, 0, 0, 0],
#     [0, 0, 0, 1, 0, 0],
# ])

# R = np.array([
#     [9, 0],
#     [0, 9],
# ])

# for i in range(10): 
#     K = P @ H.transpose() @ np.linalg.inv(H @ P @ H.transpose() + R)

#     print(K)

#     P = (I - K @ H) @ P @ (I - K @ H).transpose() + K @ R @ K.transpose()

#     # print(P)

#     P = F @ P @ F.transpose() + Q

if __name__ == '__main__':
    df = pd.read_csv('data.csv')

    X_hat = np.array([-0.6124080846091824,-1.2171945819828736,0,0,0,0]).transpose()

    old_ts = 0

    for i, row in df.iterrows():
        print(row)
        delta = row['TIMESTAMP'] - old_ts

        old_ts = row['TIMESTAMP']

        F = make_f_mat(delta / 1000)
        Q = make_q_mat(delta / 1000)

        X_hat = F @ X_hat
        P = F @ P @ F.transpose() + Q

        # velocity = calculate_velocity([
        #     row['DIRECTION_X'],
        #     row['DIRECTION_Y'],
        #     row['DIRECTION_Z']
        # ], row['SPEED'])

        # Z = np.array([
        #     row["ACCELERATION_X"],
        #     row["ACCELERATION_Y"],
        #     velocity[0],
        #     velocity[1]
        # ]).transpose()

        # print(Z)

        # K = P @ H.transpose() @ np.linalg.inv(H @ P @ H.transpose() + R)

        # X_hat = X_hat + K @ (Z - H @ X_hat)

        # P = (I - K @ H) @ P @ (I - K @ H).transpose() + K @ R @ K.transpose()

        # print(P)

        print(X_hat)

        if (int(i) >= 3):
            break
