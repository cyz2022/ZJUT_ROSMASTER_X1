#速度环
from Rosmaster_Lib import Rosmaster
import time
import math
import numpy as np
import threading
import matplotlib.pyplot as plt
bot = Rosmaster()

v_a = 0
v_b = 0
v_c = 0
v_d = 0

distance = 0


# 20 1.95(1号板)  30 2.72  50 4.47
#                   2.72



def process_speed():
    global v_a
    global v_b
    global v_c
    global v_d
    global distance
    flag = 0 
    d = 0.104
    l = math.pi*d
    a = 0
    b = 0
    c = 0
    d = 0
    a1 = 30
    b1 = 30
    c1 = 30
    d1 = 30
    list_a = []
    list_b = []
    list_c = []
    list_d = []
    start = time.time()
    distance = 0
    k = 1
    bot.reset_flash_value()
    while 1:
        bot.set_motor(a1,b1,c1,d1)
        a,b,c,d = bot.get_motor_encoder()        
        turns =a*1.0/1320
        #print(turns)
        if turns < 1 and flag == 0: 
            flag = 1
        if flag == 1:
            a = a*1.0/1320
            b = b*1.0/1320
            c = c*1.0/1320
            d = d*1.0/1320 
            distance = turns * l
            print(distance)
            if time.time() - start > 5:
                bot.set_car_motion(0,0,0)
                break

if __name__ == '__main__':
    bot.clear_auto_report_data()
    bot.reset_flash_value()
    bot.create_receive_threading()
    enable = True
    bot.set_auto_report_state(enable, forever=False)
    set_distance = 1
    start = time.time()
   # threading.Thread(target = process_speed,args = (1,)).start()
    process_speed() 
   
