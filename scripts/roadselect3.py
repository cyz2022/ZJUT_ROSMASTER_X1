#ËÙ¶È»·
from Rosmaster_Lib import Rosmaster
import time
import math
import numpy as np
import threading
import matplotlib.pyplot as plt
from imu_information import *
bot = Rosmaster()

def turn_cross():
    a1 = 100
    b1 = 100
    c1 = -100
    d1 = -100
    old_time = 0
    start = time.time()
    new_value = 0
    list_value = []
    error_a = 0
    error_b = 0
    error_c = 0
    error_d = 0
    acc_error_a = 0
    acc_error_b = 0
    acc_error_c = 0
    acc_error_d = 0
    p_m = 2
    i_m = 0.6
    p_b = 2
    i_b = 0.6		
    pwm_a = 0
    pwm_b = 0
    pwm_c = 0	
    pwm_d = 0
    set_speed = 0
    flag = 1
    r = 0	
    newnew_time = 0
    start_time = time.time()
    bot.set_motor(a1,b1,c1,d1)
    while 1:
        angle = return_angle()
        if flag == 1:
            vx,vy,vz = bot.get_motion_data()
            print("vz",vz)
            if(vz !=0):
                r = (vx/vz)
                print("r",r)
            #distance = turns * l
            error_a = (set_speed - r)*10
            error_b = (set_speed - r)*10
            error_c = (set_speed - r)*10
            error_d = (set_speed - r)*10
            print("error",error_a)
            acc_error_a = acc_error_a + error_a
            acc_error_b = acc_error_b + error_b
            acc_error_c = acc_error_c + error_c
            acc_error_d = acc_error_d + error_d
            pwm_a = p_b*error_a +i_b*acc_error_a
            pwm_b = p_m*error_b +i_m*acc_error_b
            pwm_c = p_m*error_c +i_m*acc_error_c
            pwm_d = p_b*error_d +i_b*acc_error_d
            bot.set_motor(a1+pwm_a,b1+pwm_b,c1+pwm_c,d1+pwm_d)
            print("1",a1-pwm_a,"2",b1-pwm_b,"3",c1-pwm_c,"4",d1-pwm_d)
            

        if (time.time() - start > 2):
            bot.set_car_motion(0,0,0)
            break

if __name__ == '__main__':
    turn_cross()
    
