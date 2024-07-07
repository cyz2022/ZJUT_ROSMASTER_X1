# ËÙ¶È»·
from Rosmaster_Lib import Rosmaster
from imu_information import *
import time
import math
import numpy as np
import threading
import matplotlib.pyplot as plt
from simple_input import *

bot = Rosmaster()


def process_speed_mountain(set_speed, exit_condition):
    start_time = time.time()
    count = 0
    error_a = 0
    last_error_a = 0
    acc_error_a = 0
    p_m = 2
    i_m = 0.06
    p_b = 2
    i_b = 0.06
    change_speed=20
    
    left_front = get_GPIO(11)
    right_front = get_GPIO(5)
    left_rear = get_GPIO(19)
    right_rear = get_GPIO(26)
    left_middle = get_GPIO(9)
    right_middle = get_GPIO(6)
    pwm_a = pwm_b = pwm_c = pwm_d = 0
    a1 = b1 = c1 = d1 = set_speed
    bot.set_motor(set_speed, set_speed, set_speed, set_speed)
    useless = return_angle()
    target_angle = return_angle()
    while 1:        
        left_middle = get_GPIO(9)
        right_middle = get_GPIO(6)
        pitch = return_pitch()
        print("pitch",pitch)
        now_angle = return_angle()
        error_a = target_angle - now_angle
        # print("error",error_a)
        #print("left_middle",left_middle)
        #print("right_middle",right_middle)
        if -180 < error_a < 180:
            error_b = target_angle - now_angle
            acc_error_a = acc_error_a + error_b
            pwm_a = p_b * error_b + i_b * acc_error_a
            pwm_b = p_b * error_b + i_b * acc_error_a
            pwm_c = p_m * error_b + i_m * acc_error_a
            pwm_d = p_m * error_b + i_m * acc_error_a
            if left_middle==0 and right_middle==1:
                pwm_a+=change_speed
                pwm_b+=change_speed
                pwm_c+=change_speed
                pwm_d+=change_speed
            if right_middle==0 and left_middle==1:
                pwm_c-=change_speed
                pwm_d-=change_speed
                pwm_a-=change_speed
                pwm_b-=change_speed
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)
        if error_a > 180:
            error_c = target_angle - now_angle - 360
            acc_error_a = acc_error_a + error_c
            pwm_a = p_b * error_c + i_b * acc_error_a
            pwm_b = p_b * error_c + i_b * acc_error_a
            pwm_c = p_m * error_c + i_m * acc_error_a
            pwm_d = p_m * error_c + i_m * acc_error_a
            if left_middle==0 and right_middle==1:
                pwm_a+=change_speed
                pwm_b+=change_speed
                pwm_c+=change_speed
                pwm_d+=change_speed
            if right_middle==0 and left_middle==1:
                pwm_c-=change_speed
                pwm_d-=change_speed
                pwm_a-=change_speed
                pwm_b-=change_speed
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)
        if error_a < -180:
            error_d = target_angle - now_angle + 360
            acc_error_a = acc_error_a + error_d
            pwm_a = p_b * error_d + i_b * acc_error_a
            pwm_b = p_b * error_d + i_b * acc_error_a
            pwm_c = p_m * error_d + i_m * acc_error_a
            pwm_d = p_m * error_d + i_m * acc_error_a
            if left_middle==0 and right_middle==1:
                pwm_a+=change_speed
                pwm_b+=change_speed
                pwm_c+=change_speed
                pwm_d+=change_speed
            if right_middle==0 and left_middle==1:
                pwm_c-=change_speed
                pwm_d-=change_speed
                pwm_a-=change_speed
                pwm_b-=change_speed
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)

        if pitch < -5:
            count = 1

        if eval(exit_condition):   # 退出条件
            bot.set_car_motion(0, 0, 0)
            break


def process_speed1(set_speed, exit_condition):
    start_time = time.time()
    error_a = 0
    last_error_a = 0
    acc_error_a = 0
    p_m = 2
    i_m = 0.1
    p_b = 2
    i_b = 0.1
    pwm_a = pwm_b = pwm_c = pwm_d = 0
    a1 = b1 = c1 = d1 = set_speed
    bot.set_motor(set_speed, set_speed, set_speed, set_speed)
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
            bot.set_motor(30,  -pwm_b, 10,  pwm_d)
            print("1", a1 + pwm_a, "2",  pwm_b, "3", c1 - pwm_c, "4",  pwm_d)
        if error_a > 180:
            error_c = target_angle - now_angle - 360
            acc_error_a = acc_error_a + error_c
            pwm_a = p_b * error_c + i_b * acc_error_a
            pwm_b = p_b * error_c + i_b * acc_error_a
            pwm_c = p_m * error_c + i_m * acc_error_a
            pwm_d = p_m * error_c + i_m * acc_error_a
            bot.set_motor(30, -pwm_b, 10, pwm_d)
            print("1", a1 + pwm_a, "2", pwm_b, "3", c1 - pwm_c, "4",  pwm_d)
        if error_a < -180:
            error_d = target_angle - now_angle + 360
            acc_error_a = acc_error_a + error_d
            pwm_a = p_b * error_d + i_b * acc_error_a
            pwm_b = p_b * error_d + i_b * acc_error_a
            pwm_c = p_m * error_d + i_m * acc_error_a
            pwm_d = p_m * error_d + i_m * acc_error_a
            bot.set_motor(30,  -pwm_b, 10, pwm_d)
            print("1", a1 + pwm_a, "2", pwm_b, "3", c1 - pwm_c, "4", pwm_d)

        if eval(exit_condition):
            bot.set_car_motion(0, 0, 0)
            break

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
    change_speed=50
    left_front = get_GPIO(11)
    right_front = get_GPIO(5)
    left_rear = get_GPIO(19)
    right_rear = get_GPIO(26)
    left_middle = get_GPIO(9)
    right_middle = get_GPIO(6)
    pwm_a = pwm_b = pwm_c = pwm_d = 0
    a1 = b1 = c1 = d1 = set_speed
    bot.set_motor(set_speed, set_speed, set_speed, set_speed)
    #useless = return_angle()
    #target_angle = return_angle()
    while 1:
       # a,b,c,d = bot.get_motor_encoder()
        #print(a,b,c,d)
        left_middle = get_GPIO(9)
        right_middle = get_GPIO(6)
        pitch = return_pitch()
        print("pitch",pitch)
        now_angle = return_angle()
        error_a = target_angle - now_angle
        #print("left_middle",left_middle)
        #print("right_middle",right_middle)
        # print("error",error_a)
        delta = 30
        if -180 < error_a < 180:
            error_b = target_angle - now_angle
            acc_error_a = acc_error_a + error_b
            pwm_a = p_b * error_b + i_b * acc_error_a
            pwm_b = p_b * error_b + i_b * acc_error_a
            pwm_c = p_m * error_b + i_m * acc_error_a
            pwm_d = p_m * error_b + i_m * acc_error_a
            if left_middle==0 and right_middle==1:
                if -pwm_a > change_speed:
                    pwm_a-=change_speed
                    pwm_b+=delta
                else:
                    pwm_a+=change_speed
                    if delta >change_speed:
                        pwm_b+=delta
                    else :
                        pwm_b+=change_speed

            if right_middle==0 and left_middle==1:
                pwm_c-=change_speed
                if delta >change_speed:
                    pwm_d-=delta
                else:
                    pwm_d-=change_speed
            bot.set_motor(a1 - pwm_a, b1 - pwm_b + delta, c1 + pwm_c, d1 + pwm_d + delta)
            #print("1", a1 - pwm_a, "2", b1 - pwm_b + delta, "3", c1 + pwm_c, "4", d1 + pwm_d + delta)
        if error_a > 180:
            error_c = target_angle - now_angle - 360
            acc_error_a = acc_error_a + error_c
            pwm_a = p_b * error_c + i_b * acc_error_a
            pwm_b = p_b * error_c + i_b * acc_error_a
            pwm_c = p_m * error_c + i_m * acc_error_a
            pwm_d = p_m * error_c + i_m * acc_error_a
            if left_middle==0 and right_middle==1:
                if -pwm_a > change_speed:
                    pwm_a-=change_speed
                    pwm_b+=delta
                else:
                    pwm_a+=change_speed
                    if delta >change_speed:
                        pwm_b+=delta
                    else :
                        pwm_b+=change_speed

            if right_middle==0 and left_middle==1:
                pwm_c-=change_speed
                if delta >change_speed:
                    pwm_d-=delta
                else:
                    pwm_d-=change_speed
            bot.set_motor(a1 - pwm_a, b1 - pwm_b + delta, c1 + pwm_c, d1 + pwm_d + delta)
            #print("1", a1 - pwm_a, "2", b1 - pwm_b + delta, "3", c1 + pwm_c, "4", d1 + pwm_d + delta)
        if error_a < -180:
            error_d = target_angle - now_angle + 360
            acc_error_a = acc_error_a + error_d
            pwm_a = p_b * error_d + i_b * acc_error_a
            pwm_b = p_b * error_d + i_b * acc_error_a
            pwm_c = p_m * error_d + i_m * acc_error_a
            pwm_d = p_m * error_d + i_m * acc_error_a
            if left_middle==0 and right_middle==1:
                if -pwm_a > change_speed:
                    pwm_a-=change_speed
                    pwm_b+=delta
                else:
                    pwm_a+=change_speed
                    if delta >change_speed:
                        pwm_b+=delta
                    else :
                        pwm_b+=change_speed

            if right_middle==0 and left_middle==1:
                pwm_c-=change_speed
                if delta >change_speed:
                    pwm_d-=delta
                else:
                    pwm_d-=change_speed
            bot.set_motor(a1 - pwm_a,b1 - pwm_b + delta, c1 + pwm_c, d1 + pwm_d + delta)
            #print("1", a1 - pwm_a, "2", b1 - pwm_b + delta, "3", c1 + pwm_c, "4", d1 + pwm_d + delta)
        if pitch > 40:
            print("pitch 40",pitch)
            #fail_flag = 1
            while 1:
                bot.set_motor(0,0,0,0)                
                pitch = return_pitch()
                if 0 < pitch < 5:
                    return 0
                    #break

        if pitch < -5:
            count = 1

        if eval(exit_condition):   # 退出条件
            #bot.set_car_motion(0, 0, 0)
           # break
            return 1
def slow_down(set_speed, exit_condition,target_angle):
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
    change_speed=30
    left_front = get_GPIO(11)
    right_front = get_GPIO(5)
    left_rear = get_GPIO(19)
    right_rear = get_GPIO(26)
    left_middle = get_GPIO(9)
    right_middle = get_GPIO(6)
    pwm_a = pwm_b = pwm_c = pwm_d = 0
    a1 = b1 = c1 = d1 = set_speed
    bot.set_motor(set_speed, set_speed, set_speed, set_speed)
    #useless = return_angle()
    #target_angle = return_angle()
    while 1:
       # a,b,c,d = bot.get_motor_encoder()
        #print(a,b,c,d)
        left_middle = get_GPIO(9)
        right_middle = get_GPIO(6)
        pitch = return_pitch()
        print("pitch",pitch)
        now_angle = return_angle()
        error_a = target_angle - now_angle
        #print("left_middle",left_middle)
        #print("right_middle",right_middle)
        # print("error",error_a)
        delta = 20
        if -180 < error_a < 180:
            error_b = target_angle - now_angle
            acc_error_a = acc_error_a + error_b
            pwm_a = p_b * error_b + i_b * acc_error_a
            pwm_b = p_b * error_b + i_b * acc_error_a
            pwm_c = p_m * error_b + i_m * acc_error_a
            pwm_d = p_m * error_b + i_m * acc_error_a
            if left_middle==0 and right_middle==1:
                if -pwm_a > change_speed:
                    pwm_a-=change_speed
                    pwm_b+=delta
                else:
                    pwm_a+=change_speed
                    if delta >change_speed:
                        pwm_b+=delta
                    else :
                        pwm_b+=change_speed

            if right_middle==0 and left_middle==1:
                pwm_c-=change_speed
                if delta >change_speed:
                    pwm_d-=delta
                else:
                    pwm_d-=change_speed
            bot.set_motor(a1 - pwm_a, b1 - pwm_b + delta, c1 + pwm_c, d1 + pwm_d + delta)
            #print("1", a1 - pwm_a, "2", b1 - pwm_b + delta, "3", c1 + pwm_c, "4", d1 + pwm_d + delta)
        if error_a > 180:
            error_c = target_angle - now_angle - 360
            acc_error_a = acc_error_a + error_c
            pwm_a = p_b * error_c + i_b * acc_error_a
            pwm_b = p_b * error_c + i_b * acc_error_a
            pwm_c = p_m * error_c + i_m * acc_error_a
            pwm_d = p_m * error_c + i_m * acc_error_a
            if left_middle==0 and right_middle==1:
                if -pwm_a > change_speed:
                    pwm_a-=change_speed
                    pwm_b+=delta
                else:
                    pwm_a+=change_speed
                    if delta >change_speed:
                        pwm_b+=delta
                    else :
                        pwm_b+=change_speed

            if right_middle==0 and left_middle==1:
                pwm_c-=change_speed
                if delta >change_speed:
                    pwm_d-=delta
                else:
                    pwm_d-=change_speed
            bot.set_motor(a1 - pwm_a, b1 - pwm_b + delta, c1 + pwm_c, d1 + pwm_d + delta)
            #print("1", a1 - pwm_a, "2", b1 - pwm_b + delta, "3", c1 + pwm_c, "4", d1 + pwm_d + delta)
        if error_a < -180:
            error_d = target_angle - now_angle + 360
            acc_error_a = acc_error_a + error_d
            pwm_a = p_b * error_d + i_b * acc_error_a
            pwm_b = p_b * error_d + i_b * acc_error_a
            pwm_c = p_m * error_d + i_m * acc_error_a
            pwm_d = p_m * error_d + i_m * acc_error_a
            if left_middle==0 and right_middle==1:
                if -pwm_a > change_speed:
                    pwm_a-=change_speed
                    pwm_b+=delta
                else:
                    pwm_a+=change_speed
                    if delta >change_speed:
                        pwm_b+=delta
                    else :
                        pwm_b+=change_speed

            if right_middle==0 and left_middle==1:
                pwm_c-=change_speed
                if delta >change_speed:
                    pwm_d-=delta
                else:
                    pwm_d-=change_speed
            bot.set_motor(a1 - pwm_a,b1 - pwm_b + delta, c1 + pwm_c, d1 + pwm_d + delta)
            #print("1", a1 - pwm_a, "2", b1 - pwm_b + delta, "3", c1 + pwm_c, "4", d1 + pwm_d + delta)
        if pitch > 40:
            print("pitch 40",pitch)
            #fail_flag = 1
            while 1:
                bot.set_motor(0,0,0,0)                
                pitch = return_pitch()
                if 0 < pitch < 5:
                    return 0
                    #break

        if pitch < -5:
            count = 1

        if eval(exit_condition):   # 退出条件
            #bot.set_car_motion(0, 0, 0)
           # break
            return 1

def go_t_mountain():
    flag = 0
    target_angle = 0
    while flag == 0:
        if target_angle != 0:
            correct_angle(target_angle)
        print("----------go_mountain---------------")
        process_speed_mountain(20,"time.time() - start_time > 0.5")
        useless = return_angle()
        target_angle = return_angle()
        bot.set_car_motion(0,0,0)
        process_speed_mountain(-20,"time.time() - start_time > 0.3")
        bot.set_car_motion(0,0,0)
        print("1 stage")
        flag = t_mountain(50,"pitch > 10",target_angle)
        print("2 stage")
        flag = t_mountain(70,"pitch > 20",target_angle)
        print("3 stage")
        flag = t_mountain(80,"0 < pitch < 5",target_angle)
        print(" on the t_mountain")
     
    bot.set_car_motion(0,0,0)
    time.sleep(0.2)
    print("-------------slip down----------")
    process_speed_mountain(20, "count == 1 and -1 < pitch < 5")
    bot.set_car_motion(0,0,0)
    correct_micro_angle(target_angle)
    return target_angle


if __name__ == '__main__':
    useless = return_angle()
    target_angle = return_angle()
    slow_down(55,"time.time() - start_time > 5",target_angle)
    #go_t_mountain()
    #process_speed1(-30,"time.time() - start_time > 0.8")
    #process_speed(20, "-5 < pitch < 5")







