import subprocess
from threading import Thread
from time import sleep

ids = [i for i in range(100)]

def run_thread(idx):
    print(f"Thread {idx} starting.")

    while (len(ids)):
        i = ids.pop()

        server = subprocess.Popen(["./imu-sensor-stream-linux", '-s', str(i), '-p', str(4242 + idx)],
                                stdout=subprocess.DEVNULL
                                )
        
        sleep(1)

        client = subprocess.Popen(["python3", "kalman.py", str(4242 + idx)], 
                                stdout=subprocess.DEVNULL
                                )
        
        result = server.wait()

        if (result != 0):
            print(f'[KO] {i}')
        else:
            print(f'[OK] {i}')

        client.kill()

    print(f"Thread {idx} finished.")

if __name__ == '__main__':
    for i in range(0, 8):
        t = Thread(target=run_thread, args=(i, ))

        t.start()