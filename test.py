from math import inf
import subprocess
from threading import Thread
from time import sleep

ids = [i for i in range(100)]

avg_max_diff = []
total_ok = 0
total_ko = 0


def run_thread(idx):
    global avg_max_diff, total_ko, total_ok
    print(f"Thread {idx} starting.")

    while len(ids):
        i = ids.pop()

        server = subprocess.Popen(
            [
                "./imu-sensor-stream-linux",
                "--delta",
                "-d",
                "10",
                "-s",
                str(i),
                "-p",
                str(4242 + idx),
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
        )

        sleep(1)

        client = subprocess.Popen(
            ["python3", "kalman.py", str(4242 + idx)], stdout=subprocess.DEVNULL
        )

        output = str(server.stdout.read(), "utf-8").split("\n")[2:-1]
        result = server.wait()
        client.kill()

        max_diff = -inf

        for line in output:
            splitted = line.split(",")
            x = float(splitted[0][3:])
            y = float(splitted[1][4:])
            z = float(splitted[2][4:])

            diff = abs(x) + abs(y) + abs(z)

            if diff > max_diff:
                max_diff = diff

        if result != 0:
            print(f"[KO] {i} > ({max_diff}), tot. iter: {len(output)}")
            total_ko += 1
        else:
            print(f"[OK] {i} ({max_diff}), tot. iter: {len(output)}")
            total_ok += 1

        avg_max_diff.append(max_diff)

    print(f"Thread {idx} finished.")


if __name__ == "__main__":
    threads = []
    for i in range(0, 8):
        t = Thread(target=run_thread, args=(i,))

        t.start()

        threads.append(t)

    for t in threads:
        t.join()

    print(f"Avg max_diff: {sum(avg_max_diff) / len(avg_max_diff)}")
    print(f"Tot. OK: {total_ok}")
    print(f"Tot. KO: {total_ko}")
