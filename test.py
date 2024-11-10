import math
from typing import List
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

np.set_printoptions(linewidth=100, suppress=True, precision=12)


def plot_diffs(diffs, reals, preds):
    labels = [
        "XPos",
        "XVel",
        "XAccel",
        "YPos",
        "YVel",
        "YAccel",
        "ZPos",
        "ZVel",
        "ZAccel",
    ]

    for i in range(len(labels)):
        plt.plot(diffs[i], color="blue")

        plt.title(f"Diff {labels[i]}")
        plt.savefig("./output/diff_" + labels[i] + ".png")

        plt.cla()

        plt.plot(reals[i], color="blue", alpha=0.5)
        plt.plot(preds[i], color="red", alpha=0.5)

        plt.title(f"Traj {labels[i]}")

        plt.savefig("./output/traj_" + labels[i] + ".png")

        plt.cla()


def make_f_mat(delta):
    vel = delta * delta * 0.5
    F = np.array(
        [
            [1, delta, vel, 0, 0, 0],
            [0, 1, delta, 0, 0, 0],
            [0, 0, 1, 0, 0, 0],
            [0, 0, 0, 1, delta, vel],
            [0, 0, 0, 0, 1, delta],
            [0, 0, 0, 0, 0, 1],
        ]
    )

    F = np.eye(9)

    for i in range(0, 3):
        F[i * 3][i * 3 + 1] = delta
        F[i * 3][i * 3 + 2] = vel
        F[i * 3 + 1][i * 3 + 2] = delta

    return F


def make_q_mat(F):
    Q = np.zeros((9, 9))

    for i in range(0, 3):
        Q[i * 3 + 2][i * 3 + 2] = 1

    Q = (F @ Q @ F.transpose()) * 0.001

    return Q


def calculate_velocity(direction: List[float], speed: int):
    pitch = direction[1]
    yaw = direction[2]

    return [
        speed * math.cos(pitch) * math.cos(yaw),
        speed * math.cos(pitch) * math.sin(yaw),
        -speed * math.sin(pitch),
    ]


I = np.eye(9)

R = {"acceleration": np.diag([0.002] * 3), "position": np.diag([0.02] * 3)}

P = np.eye(9)

H = {
    "acceleration": np.array(
        [
            # [0, 1, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0, 0, 0, 0],
            # [0, 0, 0, 0, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 1, 0, 0, 0],
            # [0, 0, 0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 1],
        ]
    ),
    "position": np.array(
        [
            [1, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 1, 0, 0],
        ]
    ),
}

# X_hat = np.array([
#     4.575474128329878, #XPos
#     0,
#     0,
#     -9.228122659741874, #Ypos
#     0,
#     0,
#     0.5, # Zpos
#     0,
#     0
#     ]).transpose()


def update(H: np.ndarray, R: np.ndarray, Z: np.ndarray, dt: int):
    global X_hat, P

    F = make_f_mat(dt * 1.0e-3)
    Q = make_q_mat(F)

    X_hat = F @ X_hat
    P = (F @ P @ F.transpose()) + Q

    K = P @ H.transpose() @ np.linalg.matrix_power(H @ P @ H.transpose() + R, -1)

    X_hat = X_hat + K @ (Z - H @ X_hat)

    P = (I - K @ H) @ P @ (I - K @ H).transpose() + K @ R @ K.transpose()


if __name__ == "__main__":
    df = pd.read_csv("out_3.csv")

    pos_df = pd.read_csv("out_3_position.csv")
    pos_iter = 0

    diffs = [[] for j in range(9)]
    reals = [[] for j in range(9)]
    preds = [[] for j in range(9)]

    max_diff = -math.inf

    X_hat = np.array(
        [
            4.575474128329878,  # XPos
            0,
            0,
            -9.228122659741874,  # Ypos
            0,
            0,
            0.5,  # Zpos
            0,
            0,
        ]
    ).transpose()

    old_ts = 0

    for i, row in df.iterrows():
        delta = row["TIMESTAMP"] - old_ts

        old_ts = row["TIMESTAMP"]

        velocity = calculate_velocity(
            [row["DIRECTION_X"], row["DIRECTION_Y"], row["DIRECTION_Z"]],
            row["SPEED"] / 3.6,
        )

        if i == 0:
            X_hat[1] = velocity[0]
            X_hat[4] = velocity[1]
            X_hat[7] = velocity[2]

            X_hat[2] = row["ACCELERATION_X"]
            X_hat[5] = row["ACCELERATION_Y"]
            X_hat[8] = row["ACCELERATION_Z"]

        Z = np.array(
            [
                # velocity[0],
                row["ACCELERATION_X"],
                # velocity[1],
                row["ACCELERATION_Y"],
                # velocity[2],
                row["ACCELERATION_Z"],
            ]
        ).transpose()

        update(H["acceleration"], R["acceleration"], Z, 10)

        if i != 0 and row["TIMESTAMP"] % 3000 == 0:
            Z = np.array(
                [
                    pos_df.iloc[pos_iter]["POSITION_X"],
                    pos_df.iloc[pos_iter]["POSITION_Y"],
                    pos_df.iloc[pos_iter]["POSITION_Z"],
                ]
            )

            pos_iter += 1

            update(H["position"], R["position"], Z, 0)

        real = [
            row["TRUE_POSITION_X"],
            velocity[0],
            row["ACCELERATION_X"],
            row["TRUE_POSITION_Y"],
            velocity[1],
            row["ACCELERATION_Y"],
            row["TRUE_POSITION_Z"],
            velocity[2],
            row["ACCELERATION_Z"],
        ]

        # print(f"I: {i}, TS: {row['TIMESTAMP']}, DT: {delta}")
        # print("Pred:", X_hat)
        # print("Real:", real)
        # print("Diff:", real - X_hat)
        # print("")

        total_diff = (
            abs(real[0] - X_hat[0]) + abs(real[1] - X_hat[1]) + abs(real[2] - X_hat[2])
        )

        if total_diff > max_diff:
            max_diff = total_diff

        diffs.append(real - X_hat)

        for l in range(len(real)):
            diffs[l].append(real[l] - X_hat[l])
            reals[l].append(real[l])
            preds[l].append(X_hat[l])

        if int(i) >= 30_000:
            break

    print("MAX: ", max_diff)
    plot_diffs(diffs, reals, preds)
