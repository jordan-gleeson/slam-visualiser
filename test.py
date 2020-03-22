import threading
import time

cur_time = time.time()
prev_time = time.time()

def start_timer():
    global prev_time
    cur_time = time.time()
    print(int(1/(cur_time - prev_time)))
    prev_time = cur_time
    threading.Timer(1/100, start_timer).start()

threading.Timer(1/30, start_timer).start()
while True:
    for i in range(1000000):
        for j in range(1000):
            pass
        if i % 100000 == 0:
            print("Running", i)