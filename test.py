import math
from typing import List
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

np.set_printoptions(linewidth=100, suppress=True, precision=12)

def plot_diffs(diffs):
    labels = ['XPos', "XAccel", "XVel", 'YPos', "YAccel", "YVel", 'ZPos', "ZAccel", "ZVel"]

    for i in range(len(labels)):
        items = []
        for diff in diffs:
            items.append(diff[i])
        
        plt.plot(items)

        plt.savefig(labels[i] + ".png")

        plt.cla()

def make_f_mat (delta):
    vel = delta * delta * 0.5
    F = np.array([
        [1, delta, vel,0,  0,  0],
        [0, 1, delta,  0,  0,  0],
        [0, 0, 1,  0,  0,  0],
        [0, 0, 0,  1,  delta,  vel],
        [0, 0, 0,  0,  1,  delta],
        [0, 0, 0,  0,  0,  1],    
    ])

    F = np.eye(9)

    for i in range(0, 3):
        F[i * 3][i * 3 + 1] = delta
        F[i * 3][i * 3 + 2] = vel
        F[i * 3 + 1][i * 3 + 2] = delta

    return F

def make_q_mat(delta, F):
    # Q1 = np.array([
    #     [delta ** 4 / 4, delta ** 3 / 2, delta ** 2 / 2, 0, 0, 0],
    #     [delta ** 3 / 2, delta ** 2    , delta         , 0, 0, 0],
    #     [delta ** 2 / 2, delta         , 1             , 0, 0, 0],
    #     [0, 0, 0, delta ** 4 / 4, delta ** 3 / 2, delta ** 2 / 2],
    #     [0, 0, 0, delta ** 3 / 2, delta ** 2    , delta         ],
    #     [0, 0, 0, delta ** 2 / 2, delta         , 1             ],
    # ]) * 0.04

    Q = np.zeros((9, 9))

    for i in range(0, 3):
        Q[i * 3 + 2][i * 3 + 2] = 1

    Q = ( F @ Q @ F.transpose() ) * 0.08

    return Q

def calculate_velocity(direction: List[float], speed: int):
    pitch = direction[1]
    yaw = direction[2]

    return [
        speed * math.cos(pitch) * math.cos(yaw),
        speed * math.cos(pitch) * math.sin(yaw),
        speed * math.sin(pitch),
    ]


I = np.eye(9)

R = np.diag([0.4] * 6)

P = np.eye(9)

H = np.array([
    [0, 1, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 1, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 1, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 1, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 1, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 1],
])

if __name__ == '__main__':
    df = pd.read_csv('out_3.csv')

    diffs = []

    X_hat = np.array([
        4.575474128329878, #XPos
        0,
        0,
        -9.228122659741874, #Ypos
        0,
        0,
        0.5, # Zpos
        0,
        0
        ]).transpose()

    old_ts = 0

    for i, row in df.iterrows():
        delta = row['TIMESTAMP'] - old_ts

        old_ts = row['TIMESTAMP']

        velocity = calculate_velocity([
            row['DIRECTION_X'],
            row['DIRECTION_Y'],
            row['DIRECTION_Z']
        ], row['SPEED'] / 3.6)

        if i == 0:
            X_hat[1] = velocity[0]
            X_hat[4] = velocity[1]
            X_hat[7] = velocity[2]

            X_hat[2] = row['ACCELERATION_X']
            X_hat[5] = row['ACCELERATION_Y']
            X_hat[8] = row['ACCELERATION_Z']

        F = make_f_mat(delta * 1.0e-3)
        Q = make_q_mat(delta * 1.0e-3, F)

        X_hat = F @ X_hat
        P = ( F @ P @ F.transpose() ) + Q

        Z = np.array([
            velocity[0],
            row["ACCELERATION_X"],
            velocity[1],
            row["ACCELERATION_Y"],
            velocity[2],
            row["ACCELERATION_Z"],
        ]).transpose()

        K = P @ H.transpose() @ np.linalg.matrix_power(H @ P @ H.transpose() + R, -1)

        X_hat = X_hat + K @ (Z - H @ X_hat)

        P = (I - K @ H) @ P @ (I - K @ H).transpose() + K @ R @ K.transpose()

        real = [
            row['TRUE_POSITION_X'], 
            velocity[0],
            row['ACCELERATION_X'],

            row['TRUE_POSITION_Y'],
            velocity[1],
            row['ACCELERATION_Y'],

            row['TRUE_POSITION_Z'],
            velocity[2],
            row['ACCELERATION_Z'],
            ]

        print(f"I: {i}, TS: {row['TIMESTAMP']}, DT: {delta}")
        print("K:", K)
        print("Pred:", X_hat)
        print("Real:", real)
        print("Diff:", real - X_hat)
        print("")

        diffs.append(real - X_hat)

        if (int(i) >= 50000):
            break
    plot_diffs(diffs)
