# ËÙ¶È»·
from Rosmaster_Lib import Rosmaster
from imu_information import *
import time
import math
import numpy as np
import threading
import matplotlib.pyplot as plt
from simple_input import *
from go_straight import *
bot = Rosmaster()

fail_flag = 0

def process_speed(set_speed, exit_condition):
    start_time = time.time()
    left_middle=get_GPIO(9)
    right_middle=get_GPIO(6)
    count = 0
    error_a = 0
    last_error_a = 0
    acc_error_a = 0
    p_m = 2
    i_m = 0.06
    p_b = 2
    i_b = 0.06
    change_speed=10
    pwm_a = pwm_b = pwm_c = pwm_d = 0
    a1 = b1 = c1 = d1 = set_speed
    bot.set_motor(set_speed, set_speed, set_speed, set_speed)
    useless = return_angle()
    target_angle = return_angle()
    while 1:
        pitch = return_pitch()
        now_angle = return_angle()
        error_a = target_angle - now_angle
        # print("error",error_a)
        if -180 < error_a < 180:
        
            error_b = target_angle - now_angle
            acc_error_a = acc_error_a + error_b
            pwm_a = p_b * error_b + i_b * acc_error_a
            pwm_b = p_b * error_b + i_b * acc_error_a
            pwm_c = p_m * error_b + i_m * acc_error_a
            pwm_d = p_m * error_b + i_m * acc_error_a
            if left_middle==0:
                pwm_c+=change_speed
                pwm_d+=change_speed
                print("change1",pwm_a,pwm_b,pwm_c,pwm_d)
            if right_middle==0:
                pwm_c+=change_speed
                pwm_d+=change_speed
                print("change2",pwm_a,pwm_b,pwm_c,pwm_d)
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)
        if error_a > 180:
            error_c = target_angle - now_angle - 360
            acc_error_a = acc_error_a + error_c
            pwm_a = p_b * error_c + i_b * acc_error_a
            pwm_b = p_b * error_c + i_b * acc_error_a
            pwm_c = p_m * error_c + i_m * acc_error_a
            pwm_d = p_m * error_c + i_m * acc_error_a
            if left_middle==0:
                pwm_c+=change_speed
                pwm_d+=change_speed
                print("change1",pwm_a,pwm_b,pwm_c,pwm_d)
            if right_middle==0:
                pwm_c+=change_speed
                pwm_d+=change_speed
                print("change2",pwm_a,pwm_b,pwm_c,pwm_d)
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)
        if error_a < -180:
            error_d = target_angle - now_angle + 360
            acc_error_a = acc_error_a + error_d
            pwm_a = p_b * error_d + i_b * acc_error_a
            pwm_b = p_b * error_d + i_b * acc_error_a
            pwm_c = p_m * error_d + i_m * acc_error_a
            pwm_d = p_m * error_d + i_m * acc_error_a
            if left_middle==0:
                pwm_c+=change_speed
                pwm_d+=change_speed
                print("change1",pwm_a,pwm_b,pwm_c,pwm_d)
            if right_middle==0:
                pwm_c+=change_speed
                pwm_d+=change_speed
                print("change2",pwm_a,pwm_b,pwm_c,pwm_d)
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)

        if pitch < -5:
            count = 1

        if eval(exit_condition):   # 退出条件
            bot.set_car_motion(0, 0, 0)
            break


def go_t_mountain():
    useless = return_angle()
    target_angle = return_angle()
    
    print("1 stage")
    t_mountain(50,"pitch > 7",target_angle)
    print("2 stage")
    t_mountain(20,"time.time()-start_time>1",target_angle)
    t_mountain(60,"pitch > 15",target_angle)
    print("3 stage")
    t_mountain(20,"time.time()-start_time>1",target_angle)
    #pitch1=return_pitch()
    #while pitch1>5:
    t_mountain(80,"5 > pitch",target_angle)
    #t_mountain(70,"23 > pitch",target_angle)
        #pitch1=return_pitch()
        #print(pitch1,'pitch1')
    print(" on the t_mountain")
    #bot.set_motor(0,0,0,0)
    bot.set_car_motion(0,0,0)
    time.sleep(0.3)
    #process_speed(20,"pitch ")
    print("-------------slip down----------")
    process_speed(30, "-5 < pitch < -2")
    correct_micro_angle(target_angle)



def t_mountain(set_speed, exit_condition,target_angle):
    start_time = time.time()
    success_flag = 0
    fail_flag = 0
    #m是右边，b是左边
    count = 0
    error_a = 0
    last_error_a = 0
    acc_error_a = 0
    p_m = 2.0
    i_m = 0.06
    p_b = 2.0
    i_b = 0.06
    pwm_a = pwm_b = pwm_c = pwm_d = 0
    a1 = b1 = c1 = d1 = set_speed
    bot.set_motor(set_speed, set_speed, set_speed, set_speed)
    #useless = return_angle()
    #target_angle = return_angle()
    #bot.create_receive_threading()
    #enable = True
    #bot.set_auto_report_state(enable, forever=False)
    #bot.clear_auto_report_data()
    #bot.reset_flash_value()
    while 1:
        #a,b,c,d = bot.get_motor_encoder()
        #print(a,b,c,d)
        pitch = return_pitch()
        print("pitch",pitch)
        now_angle = return_angle()
        error_a = target_angle - now_angle
        # print("error",error_a)
        delta = 30
        if -180 < error_a < 180:
            error_b = target_angle - now_angle
            acc_error_a = acc_error_a + error_b
            pwm_a = p_b * error_b + i_b * acc_error_a
            pwm_b = p_b * error_b + i_b * acc_error_a
            pwm_c = p_m * error_b + i_m * acc_error_a
            pwm_d = p_m * error_b + i_m * acc_error_a 
            bot.set_motor(a1 - pwm_a, b1 - pwm_b + delta, c1 + pwm_c, d1 + pwm_d + delta)
            print("1", a1 - pwm_a, "2", b1 - pwm_b + delta, "3", c1 + pwm_c, "4", d1 + pwm_d + delta)
        if error_a > 180:
            error_c = target_angle - now_angle - 360
            acc_error_a = acc_error_a + error_c
            pwm_a = p_b * error_c + i_b * acc_error_a
            pwm_b = p_b * error_c + i_b * acc_error_a
            pwm_c = p_m * error_c + i_m * acc_error_a
            pwm_d = p_m * error_c + i_m * acc_error_a
            bot.set_motor(a1 - pwm_a, b1 - pwm_b + delta, c1 + pwm_c, d1 + pwm_d + delta)
            print("1", a1 - pwm_a, "2", b1 - pwm_b + delta, "3", c1 + pwm_c, "4", d1 + pwm_d + delta)
        if error_a < -180:
            error_d = target_angle - now_angle + 360
            acc_error_a = acc_error_a + error_d
            pwm_a = p_b * error_d + i_b * acc_error_a
            pwm_b = p_b * error_d + i_b * acc_error_a
            pwm_c = p_m * error_d + i_m * acc_error_a
            pwm_d = p_m * error_d + i_m * acc_error_a
            bot.set_motor(a1 - pwm_a,b1 - pwm_b + delta, c1 + pwm_c, d1 + pwm_d + delta)
            print("1", a1 - pwm_a, "2", b1 - pwm_b + delta, "3", c1 + pwm_c, "4", d1 + pwm_d + delta)
        if pitch > 40:
            print("pitch 40",pitch)
            #fail_flag = 1
            while 1:
                bot.set_motor(0,0,0,0)                
                pitch = return_pitch()
                if -5 < pitch < 0:
                    break

        if pitch < -5:
            count = 1

        if eval(exit_condition):   # 退出条件
            bot.set_car_motion(0, 0, 0)
            break

def t_mountain1(set_speed, exit_condition,target_angle):
    start_time = time.time()
    success_flag = 0
    fail_flag = 0
    #m是右边，b是左边
    count = 0
    error_a = 0
    last_error_a = 0
    acc_error_a = 0
    p_m = 2.0
    i_m = 0.06
    p_b = 2.0
    i_b = 0.06
    pwm_a = pwm_b = pwm_c = pwm_d = 0
    a1 = b1 = c1 = d1 = set_speed
    bot.set_motor(set_speed, set_speed, set_speed, set_speed)
    #useless = return_angle()
    #target_angle = return_angle()
    #bot.create_receive_threading()
    #enable = True
    #bot.set_auto_report_state(enable, forever=False)
    #bot.clear_auto_report_data()
    #bot.reset_flash_value()
    while 1:
        #a,b,c,d = bot.get_motor_encoder()
        #print(a,b,c,d)
        pitch = return_pitch()
        print("pitch",pitch)
        now_angle = return_angle()
        error_a = target_angle - now_angle
        # print("error",error_a)
        delta = 20
        if -180 < error_a < 180:
            error_b = target_angle - now_angle
            acc_error_a = acc_error_a + error_b
            pwm_a = p_b * error_b + i_b * acc_error_a
            pwm_b = p_b * error_b + i_b * acc_error_a
            pwm_c = p_m * error_b + i_m * acc_error_a
            pwm_d = p_m * error_b + i_m * acc_error_a 
            bot.set_motor(a1 - pwm_a+ delta, b1 - pwm_b , c1 + pwm_c+ delta, d1 + pwm_d )
            print("1", a1 - pwm_a+ delta, "2", b1 - pwm_b , "3", c1 + pwm_c+ delta, "4", d1 + pwm_d )
        if error_a > 180:
            error_c = target_angle - now_angle - 360
            acc_error_a = acc_error_a + error_c
            pwm_a = p_b * error_c + i_b * acc_error_a
            pwm_b = p_b * error_c + i_b * acc_error_a
            pwm_c = p_m * error_c + i_m * acc_error_a
            pwm_d = p_m * error_c + i_m * acc_error_a
            bot.set_motor(a1 - pwm_a+ delta, b1 - pwm_b , c1 + pwm_c+ delta, d1 + pwm_d )
            print("1", a1 - pwm_a+ delta, "2", b1 - pwm_b , "3", c1 + pwm_c+ delta, "4", d1 + pwm_d )
        if error_a < -180:
            error_d = target_angle - now_angle + 360
            acc_error_a = acc_error_a + error_d
            pwm_a = p_b * error_d + i_b * acc_error_a
            pwm_b = p_b * error_d + i_b * acc_error_a
            pwm_c = p_m * error_d + i_m * acc_error_a
            pwm_d = p_m * error_d + i_m * acc_error_a
            bot.set_motor(a1 - pwm_a+ delta,b1 - pwm_b , c1 + pwm_c+ delta, d1 + pwm_d )
            print("1", a1 - pwm_a+ delta, "2", b1 - pwm_b , "3", c1 + pwm_c+ delta, "4", d1 + pwm_d )
        if pitch > 40:
            print("pitch 40",pitch)
            #fail_flag = 1
            while 1:
                bot.set_motor(0,0,0,0)                
                pitch = return_pitch()
                if -5 < pitch < 0:
                    break

        if pitch < -5:
            count = 1

        if eval(exit_condition):   # 退出条件
            bot.set_car_motion(0, 0, 0)
            break
if __name__ == '__main__':
    go_t_mountain()
    #while 1:
        #pitch=return_pitch()
        #print(pitch)







