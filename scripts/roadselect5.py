#?¨´?¨¨?¡¤
from Rosmaster_Lib import Rosmaster
import time
import math
import numpy as np
import threading
import matplotlib.pyplot as plt
from imu_information import *
bot = Rosmaster()

v_a = 0
v_b = 0
v_c = 0
v_d = 0

def turn_cross():
    global v_a
    global v_b
    global v_c
    global v_d
    bot.clear_auto_report_data()
    bot.reset_flash_value()
    bot.create_receive_threading()
    enable = True
    bot.set_auto_report_state(enable, forever=False)
    flag = 1 
    d = 0.104
    l = math.pi*d
    a = 0
    b = 0
    c = 0
    d = 0
    a1 = -90
    b1 = -90
    c1 = 90
    d1 = 90
    d = 22
    omega = 0
    x = 0.0
    y = 0.0
    theta = 0.0
    newnew_time = 0
    ICC_x = 0
    ICC_y = 0
    list_a = []
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
    p_m = 0.2
    i_m = 0.01
    p_b = 0.2
    i_b = 0.01		
    pwm_a = 0
    pwm_b = 0
    pwm_c = 0	
    pwm_d = 0
    set_speed = 0	
    newnew_time = 0
    start_time = time.time()
    bot.set_motor(a1,b1,c1,d1)
    while 1:
        angle = return_angle()
        new_a,new_b,new_c,new_d = bot.get_motor_encoder()
        print("mc",new_a,new_b,new_c,new_d)
        

        if flag == 1:
            vx,vy,vz = bot.get_motion_data()
            print("vz",vz)
            new_time = time.time()
            delta_t = new_time - start_time
          #  print("time",delta_t)
            old_time = new_time       
            new_a = new_a*1.0/(1320* delta_t)
            new_b = new_b*1.0/(1320* delta_t)
            new_c = new_c*1.0/(1320* delta_t)
            new_d = new_d*1.0/(1320* delta_t)  
            print("v1",new_a,"v2",new_b,"v3",new_c,"v4",new_d)
            vl = (new_a + new_b)/2
            vr = (new_c + new_d)/2
            vl = (new_a + new_b)/2
            vr = (new_c + new_d)/2
            vc = (vl + vr)/2
            print("vc",vc)
            v_x,v_y,v_z = bot.get_motion_data()
            print("vz",v_z)
            if(v_z != 0):
                l = (vr/v_z - vl/v_z)
                print("l",l)
    
            wc = (vr - vl)/22
            if(wc != 0):
                banjin = vc/wc
                print("banjin",banjin)
            print("wc",wc)
            #distance = turns * l
            error_a = (set_speed - banjin)
            error_b = (set_speed - banjin)
            error_c = (set_speed - banjin)
            error_d = (set_speed - banjin)
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
            print("1",a1+pwm_a,"2",b1+pwm_b,"3",c1+pwm_c,"4",d1+pwm_d)
            

        if (time.time() - start > 5):
            bot.set_car_motion(0,0,0)
            break




if __name__ == '__main__':
    turn_cross()
