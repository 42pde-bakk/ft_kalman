import math
import sys
from typing import List
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from udp import *

np.set_printoptions(linewidth=150, suppress=True, precision=12)


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

R = {
    "acceleration": np.diag([0.000001] * 3),
    "position": np.diag([0.1] * 3),
    "velocity": np.diag([0.01] * 3),
}

Ppos = 0
Pvel = 0.001
Pacl = 0.00001
Pmul = Pvel * Pacl

P = None


def reset_P():
    global P
    P = np.array(
        [
            [Ppos, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, Pvel, Pmul, 0, 0, 0, 0, 0, 0],
            [0, Pmul, Pacl, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, Ppos, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, Pvel, Pmul, 0, 0, 0],
            [0, 0, 0, 0, Pmul, Pacl, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, Ppos, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, Pvel, Pmul],
            [0, 0, 0, 0, 0, 0, 0, Pmul, Pacl],
        ]
    )


reset_P()

H = {
    "velocity": np.array(
        [
            [0, 1, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 1, 0],
        ]
    ),
    "acceleration": np.array(
        [
            [0, 0, 1, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 1, 0, 0, 0],
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

X_hat = None


def update(H: np.ndarray, R: np.ndarray, Z: np.ndarray, dt: int):
    global X_hat, P

    F = make_f_mat(dt * 1.0e-3)
    Q = make_q_mat(F)

    X_hat = F @ X_hat
    P = (F @ P @ F.transpose()) + Q

    K = P @ H.transpose() @ np.linalg.matrix_power(H @ P @ H.transpose() + R, -1)

    X_hat = X_hat + K @ (Z - H @ X_hat)

    P = (I - K @ H) @ P @ (I - K @ H).transpose() + K @ R @ K.transpose()

    return K


def run_udp(port):
    global X_hat

    connect(serverSocket, port)
    initial_data = init(serverSocket, port)

    velocity = calculate_velocity(
        [
            initial_data["DIRECTION"][0],
            initial_data["DIRECTION"][1],
            initial_data["DIRECTION"][2],
        ],
        initial_data["SPEED"][0] / 3.6,
    )

    X_hat = np.array(
        [
            initial_data["TRUE POSITION"][0],  # XPos
            velocity[0],
            initial_data["ACCELERATION"][0],
            initial_data["TRUE POSITION"][1],  # Ypos
            velocity[1],
            initial_data["ACCELERATION"][1],
            initial_data["TRUE POSITION"][2],  # Zpos
            velocity[2],
            initial_data["ACCELERATION"][2],
        ]
    ).transpose()

    submit(serverSocket, port, f"{X_hat[0].item()} {X_hat[3].item()} {X_hat[6].item()}")

    timestamp = 10

    for i in range(1_000_000):
        input = receive(serverSocket)

        velocity_wrong = calculate_velocity(
            [input["DIRECTION"][0], input["DIRECTION"][1], input["DIRECTION"][2]],
            initial_data["SPEED"][0] / 3.6,
        )

        if i < 10:
            Z_VEL = np.array([velocity_wrong[0], velocity_wrong[1], velocity_wrong[2]])

            K = update(H["velocity"], R["velocity"], Z_VEL, 0)

        Z = np.array(
            [
                input["ACCELERATION"][0],
                input["ACCELERATION"][1],
                input["ACCELERATION"][2],
            ]
        ).transpose()

        update(H["acceleration"], R["acceleration"], Z, 10)

        if i != 0 and (timestamp) % 3000 == 0:
            Z = np.array(
                [
                    input["POSITION"][0],
                    input["POSITION"][1],
                    input["POSITION"][2],
                ]
            )

            update(H["position"], R["position"], Z, 0)

        submit(
            serverSocket, port, f"{X_hat[0].item()} {X_hat[3].item()} {X_hat[6].item()}"
        )

        timestamp += 10

        print(i)

        if int(i) >= 539996:
            break


def run(file):
    global X_hat
    df = pd.read_csv(f"{file}.csv")

    pos_df = pd.read_csv(f"{file}_position.csv")
    pos_iter = 0

    diffs = [[] for j in range(9)]
    reals = [[] for j in range(9)]
    preds = [[] for j in range(9)]

    max_diff = -math.inf

    velocity = calculate_velocity(
        [
            df.iloc[0]["DIRECTION_X"],
            df.iloc[0]["DIRECTION_Y"],
            df.iloc[0]["DIRECTION_Z"],
        ],
        df.iloc[0]["SPEED"] / 3.6,
    )

    print("VEL", velocity)

    X_hat = np.array(
        [
            df.iloc[0]["TRUE_POSITION_X"],  # XPos
            velocity[0],
            df.iloc[0]["ACCELERATION_X"],
            df.iloc[0]["TRUE_POSITION_Y"],  # Ypos
            velocity[1],
            df.iloc[0]["ACCELERATION_Y"],
            df.iloc[0]["TRUE_POSITION_Z"],  # Zpos
            velocity[2],
            df.iloc[0]["ACCELERATION_Z"],
        ]
    ).transpose()

    old_ts = 0
    timestamp = 10

    for i, row in df.iterrows():
        delta = row["TIMESTAMP"] - old_ts

        old_ts = row["TIMESTAMP"]

        velocity = calculate_velocity(
            [row["DIRECTION_X"], row["DIRECTION_Y"], row["DIRECTION_Z"]],
            row["SPEED"] / 3.6,
        )

        velocity_wrong = calculate_velocity(
            [row["DIRECTION_X"], row["DIRECTION_Y"], row["DIRECTION_Z"]],
            df.iloc[0]["SPEED"] / 3.6,
        )

        if i < 10:
            Z_VEL = np.array([velocity_wrong[0], velocity_wrong[1], velocity_wrong[2]])

            K = update(H["velocity"], R["velocity"], Z_VEL, 0)

        Z = np.array(
            [
                row["ACCELERATION_X"],
                row["ACCELERATION_Y"],
                row["ACCELERATION_Z"],
            ]
        ).transpose()

        K = update(H["acceleration"], R["acceleration"], Z, 10)

        if i != 0 and (timestamp) % 3000 == 0:
            Z = np.array(
                [
                    pos_df.iloc[pos_iter]["POSITION_X"],
                    pos_df.iloc[pos_iter]["POSITION_Y"],
                    pos_df.iloc[pos_iter]["POSITION_Z"],
                ]
            )

            pos_iter += 1

            K_pos = update(H["position"], R["position"], Z, 000)

            # exit();

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
        # print("P:", P)
        # print("Pred:", X_hat)
        # print("Real:", real)
        # print("Diff:", real - X_hat)
        # print("")

        total_diff = (
            abs(real[0] - X_hat[0]) + abs(real[3] - X_hat[3]) + abs(real[6] - X_hat[6])
        )

        if total_diff > max_diff:
            max_diff = total_diff

        diffs.append(real - X_hat)

        for l in range(len(real)):
            diffs[l].append(real[l] - X_hat[l])
            reals[l].append(real[l])
            preds[l].append(X_hat[l])

        timestamp += 10

        if int(i) >= 1000000:
            break

    print(P)

    return (max_diff, diffs, reals, preds)


if __name__ == "__main__":
    if len(sys.argv) == 2:
        run_udp(int(sys.argv[1]))
        exit()

    files = ["out_0", "out_2", "out_3", "out_27", "out_33", "out_98", "out_99"]
    # files = ["out_98"]

    for file in files:
        max_diff, diffs, reals, preds = run(file)
        print(f" {file}, MAX: ", max_diff)
        plot_diffs(diffs, reals, preds)
        reset_P()
