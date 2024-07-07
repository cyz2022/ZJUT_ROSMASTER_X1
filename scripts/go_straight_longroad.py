# ËÙ¶È»·
from Rosmaster_Lib import Rosmaster
from imu_information import *
import time
import math
import numpy as np
import threading
import matplotlib.pyplot as plt
from simple_input import *
from roadback import *
bot = Rosmaster()


def process_speed_longroad(set_speed,exit_condition):
    start_time = time.time()
    count = 0
    error_a = 0
    last_error_a = 0
    acc_error_a = 0
    p_m = 2
    i_m = 0.06
    p_b = 2
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
    useless = return_angle()
    target_angle = return_angle()
    while 1:
        left_front = get_GPIO(11)
        right_front = get_GPIO(5)
        left_rear = get_GPIO(19)
        right_rear = get_GPIO(26)
        left_middle = get_GPIO(9)
        right_middle = get_GPIO(6)
        crush_sensor = get_GPIO(13)
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
            if left_front==0 or right_front==0:
                change_speed=20
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
            if left_rear==0 or right_rear==0:
                change_speed=50
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)
        if error_a > 180:
            error_c = target_angle - now_angle - 360
            acc_error_a = acc_error_a + error_c
            pwm_a = p_b * error_c + i_b * acc_error_a
            pwm_b = p_b * error_c + i_b * acc_error_a
            pwm_c = p_m * error_c + i_m * acc_error_a
            pwm_d = p_m * error_c + i_m * acc_error_a
            if left_front==0 or right_front==0:
                change_speed=20
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
            if left_rear==0 or right_rear==0:
                change_speed=50
            
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)
        if error_a < -180:
            error_d = target_angle - now_angle + 360
            acc_error_a = acc_error_a + error_d
            pwm_a = p_b * error_d + i_b * acc_error_a
            pwm_b = p_b * error_d + i_b * acc_error_a
            pwm_c = p_m * error_d + i_m * acc_error_a
            pwm_d = p_m * error_d + i_m * acc_error_a
            if left_front==0 or right_front==0:
                change_speed=20
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
            if left_rear==0 or right_rear==0:
                change_speed=50
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)

        if pitch < -5:
            count = 1

        if eval(exit_condition):   # 退出条件
            bot.set_motor(0, 0, 0, 0)
            break


def process_speed1_old(set_speed, exit_condition):
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

def t_mountain_old(set_speed, exit_condition,target_angle):
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
    while 1:
       # a,b,c,d = bot.get_motor_encoder()
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
            bot.set_motor(a1 - pwm_a, b1 - pwm_b + delta, c1 + pwm_c, d1 + pwm_d + delta)
            print("1", a1 - pwm_a, "2", b1 - pwm_b + delta, "3", c1 + pwm_c, "4", d1 + pwm_d + delta)
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
            bot.set_motor(a1 - pwm_a, b1 - pwm_b + delta, c1 + pwm_c, d1 + pwm_d + delta)
            print("1", a1 - pwm_a, "2", b1 - pwm_b + delta, "3", c1 + pwm_c, "4", d1 + pwm_d + delta)
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
            bot.set_motor(a1 - pwm_a,b1 - pwm_b + delta, c1 + pwm_c, d1 + pwm_d + delta)
            print("1", a1 - pwm_a, "2", b1 - pwm_b + delta, "3", c1 + pwm_c, "4", d1 + pwm_d + delta)
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

def go_t_mountain_old(target_angle):
    flag = 0
    while flag == 0:
        process_speed(35,"time.time() - start_time > 3")
        useless = return_angle()
        target_angle = return_angle()
        bot.set_car_motion(0,0,0)
        process_speed(-30,"time.time() - start_time > 0.3")
        bot.set_car_motion(0,0,0)
        print("1 stage")
        flag = t_mountain(30,"pitch > 10",target_angle)
        print("2 stage")
        flag = t_mountain(70,"pitch > 20",target_angle)
        print("3 stage")
        flag = t_mountain(80,"0 < pitch < 5",target_angle)
        print(" on the t_mountain")
    #bot.set_motor(0,0,0,0)
    bot.set_car_motion(0,0,0)
    time.sleep(0.3)
    #process_speed(20,"pitch ")
    print("-------------slip down----------")
    process_speed(30, "-5 < pitch < -2")
    bot.set_car_motion(0,0,0)
    correct_micro_angle(target_angle)


def process_speed3_old(set_speed, exit_condition):
    start_time = time.time()
    count = 0
    error_a = 0
    last_error_a = 0
    acc_error_a = 0
    p_m = 2
    i_m = 0.06
    p_b = 2
    i_b = 0.06
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
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)
        if error_a > 180:
            error_c = target_angle - now_angle - 360
            acc_error_a = acc_error_a + error_c
            pwm_a = p_b * error_c + i_b * acc_error_a
            pwm_b = p_b * error_c + i_b * acc_error_a
            pwm_c = p_m * error_c + i_m * acc_error_a
            pwm_d = p_m * error_c + i_m * acc_error_a
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)
        if error_a < -180:
            error_d = target_angle - now_angle + 360
            acc_error_a = acc_error_a + error_d
            pwm_a = p_b * error_d + i_b * acc_error_a
            pwm_b = p_b * error_d + i_b * acc_error_a
            pwm_c = p_m * error_d + i_m * acc_error_a
            pwm_d = p_m * error_d + i_m * acc_error_a
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)

        if pitch < -5:
            count = 1

        if eval(exit_condition):   # 退出条件
            bot.set_car_motion(0, 0, 0)
            break

def process_speed_longroad_new_43(set_speed):
    start_time = time.time()
    count = 0
    error_a = 0
    last_error_a = 0
    acc_error_a = 0
    front_condition = 0
    middle_condition = 0
    p_m = 2
    i_m = 0.06
    p_b = 2
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
    
    useless = return_angle()
    target_angle = return_angle()
    bot.set_motor(set_speed, set_speed, set_speed, set_speed)
    while 1:
        left_front = get_GPIO(11)
        right_front = get_GPIO(5)
        left_rear = get_GPIO(19)
        right_rear = get_GPIO(26)
        left_middle = get_GPIO(9)
        right_middle = get_GPIO(6)
        crush_sensor = get_GPIO(13)
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
            if left_front==0 or right_front==0:
                change_speed=20
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
            if left_front==0 or right_front==0:
                change_speed=20
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
            if left_front==0 or right_front==0:
                change_speed=20
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

        if left_front==0 or right_front==0:
            front_condition +=1
        if left_middle==0 or right_middle==0:
            middle_condition +=1
        if front_condition > 0 and middle_condition >0:
            bot.set_car_motion(0,0,0)
            break

        #if eval(exit_condition):   # 退出条件
            #bot.set_motor(0, 0, 0, 0)
            #breakdef process_speed_longroad_new(set_speed):


def process_speed_longroad_new_46(set_speed):
    start_time = time.time()
    count = 0
    error_a = 0
    last_error_a = 0
    acc_error_a = 0
    front_condition = 0
    middle_condition = 0
    p_m = 2
    i_m = 0.06
    p_b = 2
    i_b = 0.06
    change_speed = 50

    left_front = get_GPIO(11)
    right_front = get_GPIO(5)
    left_rear = get_GPIO(19)
    right_rear = get_GPIO(26)
    left_middle = get_GPIO(9)
    right_middle = get_GPIO(6)
    pwm_a = pwm_b = pwm_c = pwm_d = 0
    a1 = b1 = c1 = d1 = set_speed

    useless = return_angle()
    target_angle = return_angle()
    bot.set_motor(set_speed, set_speed, set_speed, set_speed)
    while 1:
        left_front = get_GPIO(11)
        right_front = get_GPIO(5)
        left_rear = get_GPIO(19)
        right_rear = get_GPIO(26)
        left_middle = get_GPIO(9)
        right_middle = get_GPIO(6)
        crush_sensor = get_GPIO(13)
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
            if left_front == 0 or right_front == 0:
                change_speed = 20
            if left_middle == 0 and right_middle == 1:
                pwm_a += change_speed
                pwm_b += change_speed
                pwm_c += change_speed
                pwm_d += change_speed
            if right_middle == 0 and left_middle == 1:
                pwm_c -= change_speed
                pwm_d -= change_speed
                pwm_a -= change_speed
                pwm_b -= change_speed

            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)
        if error_a > 180:
            error_c = target_angle - now_angle - 360
            acc_error_a = acc_error_a + error_c
            pwm_a = p_b * error_c + i_b * acc_error_a
            pwm_b = p_b * error_c + i_b * acc_error_a
            pwm_c = p_m * error_c + i_m * acc_error_a
            pwm_d = p_m * error_c + i_m * acc_error_a
            if left_front == 0 or right_front == 0:
                change_speed = 20
            if left_middle == 0 and right_middle == 1:
                pwm_a += change_speed
                pwm_b += change_speed
                pwm_c += change_speed
                pwm_d += change_speed
            if right_middle == 0 and left_middle == 1:
                pwm_c -= change_speed
                pwm_d -= change_speed
                pwm_a -= change_speed
                pwm_b -= change_speed

            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)
        if error_a < -180:
            error_d = target_angle - now_angle + 360
            acc_error_a = acc_error_a + error_d
            pwm_a = p_b * error_d + i_b * acc_error_a
            pwm_b = p_b * error_d + i_b * acc_error_a
            pwm_c = p_m * error_d + i_m * acc_error_a
            pwm_d = p_m * error_d + i_m * acc_error_a
            if left_front == 0 or right_front == 0:
                change_speed = 20
            if left_middle == 0 and right_middle == 1:
                pwm_a += change_speed
                pwm_b += change_speed
                pwm_c += change_speed
                pwm_d += change_speed
            if right_middle == 0 and left_middle == 1:
                pwm_c -= change_speed
                pwm_d -= change_speed
                pwm_a -= change_speed
                pwm_b -= change_speed

            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)

        if  right_front == 0:
            front_condition += 1
        if  right_middle == 0:
            middle_condition += 1
        if front_condition > 0 and middle_condition > 0:
            bot.set_car_motion(0,0,0)
            break
def process_speed_longroad_new_47(set_speed):
    start_time = time.time()
    count = 0
    error_a = 0
    last_error_a = 0
    acc_error_a = 0
    front_condition = 0
    middle_condition = 0
    p_m = 2
    i_m = 0.06
    p_b = 2
    i_b = 0.06
    change_speed = 50

    left_front = get_GPIO(11)
    right_front = get_GPIO(5)
    left_rear = get_GPIO(19)
    right_rear = get_GPIO(26)
    left_middle = get_GPIO(9)
    right_middle = get_GPIO(6)
    pwm_a = pwm_b = pwm_c = pwm_d = 0
    a1 = b1 = c1 = d1 = set_speed

    useless = return_angle()
    target_angle = return_angle()
    bot.set_motor(set_speed, set_speed, set_speed, set_speed)
    while 1:
        left_front = get_GPIO(11)
        right_front = get_GPIO(5)
        left_rear = get_GPIO(19)
        right_rear = get_GPIO(26)
        left_middle = get_GPIO(9)
        right_middle = get_GPIO(6)
        crush_sensor = get_GPIO(13)
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
            if left_front == 0 or right_front == 0:
                change_speed = 20
            if left_middle == 0 and right_middle == 1:
                pwm_a += change_speed
                pwm_b += change_speed
                pwm_c += change_speed
                pwm_d += change_speed
            if right_middle == 0 and left_middle == 1:
                pwm_c -= change_speed
                pwm_d -= change_speed
                pwm_a -= change_speed
                pwm_b -= change_speed

            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)
        if error_a > 180:
            error_c = target_angle - now_angle - 360
            acc_error_a = acc_error_a + error_c
            pwm_a = p_b * error_c + i_b * acc_error_a
            pwm_b = p_b * error_c + i_b * acc_error_a
            pwm_c = p_m * error_c + i_m * acc_error_a
            pwm_d = p_m * error_c + i_m * acc_error_a
            if left_front == 0 or right_front == 0:
                change_speed = 20
            if left_middle == 0 and right_middle == 1:
                pwm_a += change_speed
                pwm_b += change_speed
                pwm_c += change_speed
                pwm_d += change_speed
            if right_middle == 0 and left_middle == 1:
                pwm_c -= change_speed
                pwm_d -= change_speed
                pwm_a -= change_speed
                pwm_b -= change_speed

            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)
        if error_a < -180:
            error_d = target_angle - now_angle + 360
            acc_error_a = acc_error_a + error_d
            pwm_a = p_b * error_d + i_b * acc_error_a
            pwm_b = p_b * error_d + i_b * acc_error_a
            pwm_c = p_m * error_d + i_m * acc_error_a
            pwm_d = p_m * error_d + i_m * acc_error_a
            if left_front == 0 or right_front == 0:
                change_speed = 20
            if left_middle == 0 and right_middle == 1:
                pwm_a += change_speed
                pwm_b += change_speed
                pwm_c += change_speed
                pwm_d += change_speed
            if right_middle == 0 and left_middle == 1:
                pwm_c -= change_speed
                pwm_d -= change_speed
                pwm_a -= change_speed
                pwm_b -= change_speed

            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)

        if left_front == 0  :
            front_condition += 1
        if right_front == 0:
            middle_condition += 1
        if front_condition > 0 and middle_condition > 0:
            bot.set_car_motion(0,0,0)
            break
def process_speed_longroad_new_45(set_speed):
    start_time = time.time()
    count = 0
    error_a = 0
    last_error_a = 0
    acc_error_a = 0
    front_condition = 0
    middle_condition = 0
    p_m = 2
    i_m = 0.06
    p_b = 2
    i_b = 0.06
    change_speed = 50

    left_front = get_GPIO(11)
    right_front = get_GPIO(5)
    left_rear = get_GPIO(19)
    right_rear = get_GPIO(26)
    left_middle = get_GPIO(9)
    right_middle = get_GPIO(6)
    pwm_a = pwm_b = pwm_c = pwm_d = 0
    a1 = b1 = c1 = d1 = set_speed

    useless = return_angle()
    target_angle = return_angle()
    bot.set_motor(set_speed, set_speed, set_speed, set_speed)
    while 1:
        left_front = get_GPIO(11)
        right_front = get_GPIO(5)
        left_rear = get_GPIO(19)
        right_rear = get_GPIO(26)
        left_middle = get_GPIO(9)
        right_middle = get_GPIO(6)
        crush_sensor = get_GPIO(13)
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
            if left_front == 0 or right_front == 0:
                change_speed = 20
            if left_middle == 0 and right_middle == 1:
                pwm_a += change_speed
                pwm_b += change_speed
                pwm_c += change_speed
                pwm_d += change_speed
            if right_middle == 0 and left_middle == 1:
                pwm_c -= change_speed
                pwm_d -= change_speed
                pwm_a -= change_speed
                pwm_b -= change_speed

            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)
        if error_a > 180:
            error_c = target_angle - now_angle - 360
            acc_error_a = acc_error_a + error_c
            pwm_a = p_b * error_c + i_b * acc_error_a
            pwm_b = p_b * error_c + i_b * acc_error_a
            pwm_c = p_m * error_c + i_m * acc_error_a
            pwm_d = p_m * error_c + i_m * acc_error_a
            if left_front == 0 or right_front == 0:
                change_speed = 20
            if left_middle == 0 and right_middle == 1:
                pwm_a += change_speed
                pwm_b += change_speed
                pwm_c += change_speed
                pwm_d += change_speed
            if right_middle == 0 and left_middle == 1:
                pwm_c -= change_speed
                pwm_d -= change_speed
                pwm_a -= change_speed
                pwm_b -= change_speed

            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)
        if error_a < -180:
            error_d = target_angle - now_angle + 360
            acc_error_a = acc_error_a + error_d
            pwm_a = p_b * error_d + i_b * acc_error_a
            pwm_b = p_b * error_d + i_b * acc_error_a
            pwm_c = p_m * error_d + i_m * acc_error_a
            pwm_d = p_m * error_d + i_m * acc_error_a
            if left_front == 0 or right_front == 0:
                change_speed = 20
            if left_middle == 0 and right_middle == 1:
                pwm_a += change_speed
                pwm_b += change_speed
                pwm_c += change_speed
                pwm_d += change_speed
            if right_middle == 0 and left_middle == 1:
                pwm_c -= change_speed
                pwm_d -= change_speed
                pwm_a -= change_speed
                pwm_b -= change_speed

            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)

        if left_front == 0 or right_front == 0:
            front_condition += 1
        if left_middle == 0 or right_middle == 0:
            middle_condition += 1
        if front_condition > 0 and middle_condition > 0:

            break
def process_speed_longroad_new_413(set_speed):
    start_time = time.time()
    count = 0
    error_a = 0
    last_error_a = 0
    acc_error_a = 0
    front_condition = 0
    middle_condition = 0
    p_m = 2
    i_m = 0.06
    p_b = 2
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
    
    useless = return_angle()
    target_angle = return_angle()
    bot.set_motor(set_speed, set_speed, set_speed, set_speed)
    while 1:
        left_front = get_GPIO(11)
        right_front = get_GPIO(5)
        left_rear = get_GPIO(19)
        right_rear = get_GPIO(26)
        left_middle = get_GPIO(9)
        right_middle = get_GPIO(6)
        crush_sensor = get_GPIO(13)
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
            if left_front==0 or right_front==0:
                change_speed=20
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
            if left_rear==0 or right_rear==0:
                change_speed=50
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)
        if error_a > 180:
            error_c = target_angle - now_angle - 360
            acc_error_a = acc_error_a + error_c
            pwm_a = p_b * error_c + i_b * acc_error_a
            pwm_b = p_b * error_c + i_b * acc_error_a
            pwm_c = p_m * error_c + i_m * acc_error_a
            pwm_d = p_m * error_c + i_m * acc_error_a
            if left_front==0 or right_front==0:
                change_speed=20
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
            if left_rear==0 or right_rear==0:
                change_speed=50
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)
        if error_a < -180:
            error_d = target_angle - now_angle + 360
            acc_error_a = acc_error_a + error_d
            pwm_a = p_b * error_d + i_b * acc_error_a
            pwm_b = p_b * error_d + i_b * acc_error_a
            pwm_c = p_m * error_d + i_m * acc_error_a
            pwm_d = p_m * error_d + i_m * acc_error_a
            if left_front==0 or right_front==0:
                change_speed=20
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
            if left_rear==0 or right_rear==0:
                change_speed=50
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)

        if left_front==0 or right_front==0:
            front_condition +=1
        if left_middle==0 or right_middle==0:
            middle_condition +=1
        if front_condition > 0 and middle_condition >0:
            
            break


def process_speed_longroad_new_411(set_speed):
    start_time = time.time()
    count = 0
    error_a = 0
    last_error_a = 0
    acc_error_a = 0
    front_condition = 0
    middle_condition = 0
    p_m = 2
    i_m = 0.06
    p_b = 2
    i_b = 0.06
    change_speed = 50

    left_front = get_GPIO(11)
    right_front = get_GPIO(5)
    left_rear = get_GPIO(19)
    right_rear = get_GPIO(26)
    left_middle = get_GPIO(9)
    right_middle = get_GPIO(6)
    pwm_a = pwm_b = pwm_c = pwm_d = 0
    a1 = b1 = c1 = d1 = set_speed

    useless = return_angle()
    target_angle = return_angle()
    bot.set_motor(set_speed, set_speed, set_speed, set_speed)
    while 1:
        left_front = get_GPIO(11)
        right_front = get_GPIO(5)
        left_rear = get_GPIO(19)
        right_rear = get_GPIO(26)
        left_middle = get_GPIO(9)
        right_middle = get_GPIO(6)
        crush_sensor = get_GPIO(13)
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
            if left_front == 0 or right_front == 0:
                change_speed = 20
            if left_middle == 0 and right_middle == 1:
                pwm_a += change_speed
                pwm_b += change_speed
                pwm_c += change_speed
                pwm_d += change_speed
            if right_middle == 0 and left_middle == 1:
                pwm_c -= change_speed
                pwm_d -= change_speed
                pwm_a -= change_speed
                pwm_b -= change_speed
            if left_rear == 0 or right_rear == 0:
                change_speed = 50
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)
        if error_a > 180:
            error_c = target_angle - now_angle - 360
            acc_error_a = acc_error_a + error_c
            pwm_a = p_b * error_c + i_b * acc_error_a
            pwm_b = p_b * error_c + i_b * acc_error_a
            pwm_c = p_m * error_c + i_m * acc_error_a
            pwm_d = p_m * error_c + i_m * acc_error_a
            if left_front == 0 or right_front == 0:
                change_speed = 20
            if left_middle == 0 and right_middle == 1:
                pwm_a += change_speed
                pwm_b += change_speed
                pwm_c += change_speed
                pwm_d += change_speed
            if right_middle == 0 and left_middle == 1:
                pwm_c -= change_speed
                pwm_d -= change_speed
                pwm_a -= change_speed
                pwm_b -= change_speed
            if left_rear == 0 or right_rear == 0:
                change_speed = 50
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)
        if error_a < -180:
            error_d = target_angle - now_angle + 360
            acc_error_a = acc_error_a + error_d
            pwm_a = p_b * error_d + i_b * acc_error_a
            pwm_b = p_b * error_d + i_b * acc_error_a
            pwm_c = p_m * error_d + i_m * acc_error_a
            pwm_d = p_m * error_d + i_m * acc_error_a
            if left_front == 0 or right_front == 0:
                change_speed = 20
            if left_middle == 0 and right_middle == 1:
                pwm_a += change_speed
                pwm_b += change_speed
                pwm_c += change_speed
                pwm_d += change_speed
            if right_middle == 0 and left_middle == 1:
                pwm_c -= change_speed
                pwm_d -= change_speed
                pwm_a -= change_speed
                pwm_b -= change_speed
            if left_rear == 0 or right_rear == 0:
                change_speed = 50
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)

        if left_front == 0 or right_front == 0:
            front_condition += 1
        if left_middle == 0 or right_middle == 0:
            middle_condition += 1
        if front_condition > 0 and middle_condition > 0:
            bot.set_car_motion(0,0,0)
            break
def process_speed_longroad_new_520(set_speed):
    start_time = time.time()
    count = 0
    error_a = 0
    last_error_a = 0
    acc_error_a = 0
    front_condition = 0
    middle_condition = 0
    p_m = 2
    i_m = 0.06
    p_b = 2
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
    
    useless = return_angle()
    target_angle = return_angle()
    bot.set_motor(set_speed, set_speed, set_speed, set_speed)
    while 1:
        left_front = get_GPIO(11)
        right_front = get_GPIO(5)
        left_rear = get_GPIO(19)
        right_rear = get_GPIO(26)
        left_middle = get_GPIO(9)
        right_middle = get_GPIO(6)
        crush_sensor = get_GPIO(13)
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
            if left_front==0 or right_front==0:
                change_speed=20
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
            if left_rear==0 or right_rear==0:
                change_speed=50
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)
        if error_a > 180:
            error_c = target_angle - now_angle - 360
            acc_error_a = acc_error_a + error_c
            pwm_a = p_b * error_c + i_b * acc_error_a
            pwm_b = p_b * error_c + i_b * acc_error_a
            pwm_c = p_m * error_c + i_m * acc_error_a
            pwm_d = p_m * error_c + i_m * acc_error_a
            if left_front==0 or right_front==0:
                change_speed=20
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
            if left_rear==0 or right_rear==0:
                change_speed=50
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)
        if error_a < -180:
            error_d = target_angle - now_angle + 360
            acc_error_a = acc_error_a + error_d
            pwm_a = p_b * error_d + i_b * acc_error_a
            pwm_b = p_b * error_d + i_b * acc_error_a
            pwm_c = p_m * error_d + i_m * acc_error_a
            pwm_d = p_m * error_d + i_m * acc_error_a
            if left_front==0 or right_front==0:
                change_speed=20
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
            if left_rear==0 or right_rear==0:
                change_speed=50
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)

        if  left_front==0:
            front_condition +=1
        if  left_middle==0:
            middle_condition +=1
        if front_condition > 0 and middle_condition >0:
            bot.set_car_motion(0,0,0)
            process_speed_back_road(-30,"time.time()-start_time > 0.4")
            break
def process_speed_longroad_new_530(set_speed):
    start_time = time.time()
    count = 0
    error_a = 0
    last_error_a = 0
    acc_error_a = 0
    front_condition = 0
    middle_condition = 0
    p_m = 2
    i_m = 0.06
    p_b = 2
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
    
    useless = return_angle()
    target_angle = return_angle()
    bot.set_motor(set_speed, set_speed, set_speed, set_speed)
    while 1:
        left_front = get_GPIO(11)
        right_front = get_GPIO(5)
        left_rear = get_GPIO(19)
        right_rear = get_GPIO(26)
        left_middle = get_GPIO(9)
        right_middle = get_GPIO(6)
        crush_sensor = get_GPIO(13)
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
            if left_front==0 or right_front==0:
                change_speed=20
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
            if left_rear==0 or right_rear==0:
                change_speed=50
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)
        if error_a > 180:
            error_c = target_angle - now_angle - 360
            acc_error_a = acc_error_a + error_c
            pwm_a = p_b * error_c + i_b * acc_error_a
            pwm_b = p_b * error_c + i_b * acc_error_a
            pwm_c = p_m * error_c + i_m * acc_error_a
            pwm_d = p_m * error_c + i_m * acc_error_a
            if left_front==0 or right_front==0:
                change_speed=20
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
            if left_rear==0 or right_rear==0:
                change_speed=50
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)
        if error_a < -180:
            error_d = target_angle - now_angle + 360
            acc_error_a = acc_error_a + error_d
            pwm_a = p_b * error_d + i_b * acc_error_a
            pwm_b = p_b * error_d + i_b * acc_error_a
            pwm_c = p_m * error_d + i_m * acc_error_a
            pwm_d = p_m * error_d + i_m * acc_error_a
            if left_front==0 or right_front==0:
                change_speed=20
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
            if left_rear==0 or right_rear==0:
                change_speed=50
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)

        if  left_front==0:
            front_condition +=1
        if  left_middle==0:
            middle_condition +=1
        if front_condition > 0 and middle_condition >0:
            bot.set_car_motion(0,0,0)
            process_speed_back_road(-20,"time.time()-start_time > 0.6")
            break
def process_speed_longroad_new_540(set_speed):
    start_time = time.time()
    count = 0
    error_a = 0
    last_error_a = 0
    acc_error_a = 0
    front_condition = 0
    middle_condition = 0
    p_m = 2
    i_m = 0.06
    p_b = 2
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
    
    useless = return_angle()
    target_angle = return_angle()
    bot.set_motor(set_speed, set_speed, set_speed, set_speed)
    while 1:
        left_front = get_GPIO(11)
        right_front = get_GPIO(5)
        left_rear = get_GPIO(19)
        right_rear = get_GPIO(26)
        left_middle = get_GPIO(9)
        right_middle = get_GPIO(6)
        crush_sensor = get_GPIO(13)
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
            if left_front==0 or right_front==0:
                change_speed=20
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
            if left_rear==0 or right_rear==0:
                change_speed=50
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)
        if error_a > 180:
            error_c = target_angle - now_angle - 360
            acc_error_a = acc_error_a + error_c
            pwm_a = p_b * error_c + i_b * acc_error_a
            pwm_b = p_b * error_c + i_b * acc_error_a
            pwm_c = p_m * error_c + i_m * acc_error_a
            pwm_d = p_m * error_c + i_m * acc_error_a
            if left_front==0 or right_front==0:
                change_speed=20
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
            if left_rear==0 or right_rear==0:
                change_speed=50
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)
        if error_a < -180:
            error_d = target_angle - now_angle + 360
            acc_error_a = acc_error_a + error_d
            pwm_a = p_b * error_d + i_b * acc_error_a
            pwm_b = p_b * error_d + i_b * acc_error_a
            pwm_c = p_m * error_d + i_m * acc_error_a
            pwm_d = p_m * error_d + i_m * acc_error_a
            if left_front==0 or right_front==0:
                change_speed=20
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
            if left_rear==0 or right_rear==0:
                change_speed=50
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)

        if  right_front==0:
            front_condition +=1
        if  right_middle==0:
            middle_condition +=1
        if front_condition > 0 and middle_condition >0:
            bot.set_car_motion(0,0,0)
            process_speed_back_road(-20,"time.time()-start_time > 0.6")
            break


def process_speed_longroad_new_630(set_speed):
    start_time = time.time()
    count = 0
    error_a = 0
    last_error_a = 0
    acc_error_a = 0
    front_condition = 0
    middle_condition = 0
    p_m = 2
    i_m = 0.06
    p_b = 2
    i_b = 0.06
    change_speed = 50

    left_front = get_GPIO(11)
    right_front = get_GPIO(5)
    left_rear = get_GPIO(19)
    right_rear = get_GPIO(26)
    left_middle = get_GPIO(9)
    right_middle = get_GPIO(6)
    pwm_a = pwm_b = pwm_c = pwm_d = 0
    a1 = b1 = c1 = d1 = set_speed

    useless = return_angle()
    target_angle = return_angle()
    bot.set_motor(set_speed, set_speed, set_speed, set_speed)
    while 1:
        left_front = get_GPIO(11)
        right_front = get_GPIO(5)
        left_rear = get_GPIO(19)
        right_rear = get_GPIO(26)
        left_middle = get_GPIO(9)
        right_middle = get_GPIO(6)
        crush_sensor = get_GPIO(13)
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
            if left_front == 0 or right_front == 0:
                change_speed = 20
            if left_middle == 0 and right_middle == 1:
                pwm_a += change_speed
                pwm_b += change_speed
                pwm_c += change_speed
                pwm_d += change_speed
            if right_middle == 0 and left_middle == 1:
                pwm_c -= change_speed
                pwm_d -= change_speed
                pwm_a -= change_speed
                pwm_b -= change_speed
            if left_rear == 0 or right_rear == 0:
                change_speed = 50
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)
        if error_a > 180:
            error_c = target_angle - now_angle - 360
            acc_error_a = acc_error_a + error_c
            pwm_a = p_b * error_c + i_b * acc_error_a
            pwm_b = p_b * error_c + i_b * acc_error_a
            pwm_c = p_m * error_c + i_m * acc_error_a
            pwm_d = p_m * error_c + i_m * acc_error_a
            if left_front == 0 or right_front == 0:
                change_speed = 20
            if left_middle == 0 and right_middle == 1:
                pwm_a += change_speed
                pwm_b += change_speed
                pwm_c += change_speed
                pwm_d += change_speed
            if right_middle == 0 and left_middle == 1:
                pwm_c -= change_speed
                pwm_d -= change_speed
                pwm_a -= change_speed
                pwm_b -= change_speed
            if left_rear == 0 or right_rear == 0:
                change_speed = 50
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)
        if error_a < -180:
            error_d = target_angle - now_angle + 360
            acc_error_a = acc_error_a + error_d
            pwm_a = p_b * error_d + i_b * acc_error_a
            pwm_b = p_b * error_d + i_b * acc_error_a
            pwm_c = p_m * error_d + i_m * acc_error_a
            pwm_d = p_m * error_d + i_m * acc_error_a
            if left_front == 0 or right_front == 0:
                change_speed = 20
            if left_middle == 0 and right_middle == 1:
                pwm_a += change_speed
                pwm_b += change_speed
                pwm_c += change_speed
                pwm_d += change_speed
            if right_middle == 0 and left_middle == 1:
                pwm_c -= change_speed
                pwm_d -= change_speed
                pwm_a -= change_speed
                pwm_b -= change_speed
            if left_rear == 0 or right_rear == 0:
                change_speed = 50
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)

        if right_front == 0:
            front_condition += 1
        if right_middle == 0:
            middle_condition += 1
        if front_condition > 0 and middle_condition > 0:

            break

def process_speed_longroad_new_550(set_speed):
    start_time = time.time()
    count = 0
    error_a = 0
    last_error_a = 0
    acc_error_a = 0
    front_condition = 0
    middle_condition = 0
    p_m = 2
    i_m = 0.06
    p_b = 2
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
    
    useless = return_angle()
    target_angle = return_angle()
    bot.set_motor(set_speed, set_speed, set_speed, set_speed)
    while 1:
        left_front = get_GPIO(11)
        right_front = get_GPIO(5)
        left_rear = get_GPIO(19)
        right_rear = get_GPIO(26)
        left_middle = get_GPIO(9)
        right_middle = get_GPIO(6)
        crush_sensor = get_GPIO(13)
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
            if left_front==0 or right_front==0:
                change_speed=20
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
            if left_rear==0 or right_rear==0:
                change_speed=50
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)
        if error_a > 180:
            error_c = target_angle - now_angle - 360
            acc_error_a = acc_error_a + error_c
            pwm_a = p_b * error_c + i_b * acc_error_a
            pwm_b = p_b * error_c + i_b * acc_error_a
            pwm_c = p_m * error_c + i_m * acc_error_a
            pwm_d = p_m * error_c + i_m * acc_error_a
            if left_front==0 or right_front==0:
                change_speed=20
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
            if left_rear==0 or right_rear==0:
                change_speed=50
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)
        if error_a < -180:
            error_d = target_angle - now_angle + 360
            acc_error_a = acc_error_a + error_d
            pwm_a = p_b * error_d + i_b * acc_error_a
            pwm_b = p_b * error_d + i_b * acc_error_a
            pwm_c = p_m * error_d + i_m * acc_error_a
            pwm_d = p_m * error_d + i_m * acc_error_a
            if left_front==0 or right_front==0:
                change_speed=20
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
            if left_rear==0 or right_rear==0:
                change_speed=50
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)

        if  right_front==0 or left_front==0:
            front_condition +=1
        if  right_middle==0 or left_middle==0:
            middle_condition +=1
        if front_condition > 0 and middle_condition >0:
            bot.set_car_motion(0,0,0)
            #process_speed_back_road(20,"time.time()-start_time > 0.7")
            break


def process_speed_longroad_new_right_stop(set_speed):
    start_time = time.time()
    count = 0
    error_a = 0
    last_error_a = 0
    acc_error_a = 0
    front_condition = 0
    middle_condition = 0
    p_m = 2
    i_m = 0.06
    p_b = 2
    i_b = 0.06
    change_speed = 50

    left_front = get_GPIO(11)
    right_front = get_GPIO(5)
    left_rear = get_GPIO(19)
    right_rear = get_GPIO(26)
    left_middle = get_GPIO(9)
    right_middle = get_GPIO(6)
    pwm_a = pwm_b = pwm_c = pwm_d = 0
    a1 = b1 = c1 = d1 = set_speed

    useless = return_angle()
    target_angle = return_angle()
    bot.set_motor(set_speed, set_speed, set_speed, set_speed)
    while 1:
        left_front = get_GPIO(11)
        right_front = get_GPIO(5)
        left_rear = get_GPIO(19)
        right_rear = get_GPIO(26)
        left_middle = get_GPIO(9)
        right_middle = get_GPIO(6)
        crush_sensor = get_GPIO(13)
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
            if left_front == 0 or right_front == 0:
                change_speed = 20
            if left_middle == 0 and right_middle == 1:
                pwm_a += change_speed
                pwm_b += change_speed
                pwm_c += change_speed
                pwm_d += change_speed
            if right_middle == 0 and left_middle == 1:
                pwm_c -= change_speed
                pwm_d -= change_speed
                pwm_a -= change_speed
                pwm_b -= change_speed
            if left_rear == 0 or right_rear == 0:
                change_speed = 50
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)
        if error_a > 180:
            error_c = target_angle - now_angle - 360
            acc_error_a = acc_error_a + error_c
            pwm_a = p_b * error_c + i_b * acc_error_a
            pwm_b = p_b * error_c + i_b * acc_error_a
            pwm_c = p_m * error_c + i_m * acc_error_a
            pwm_d = p_m * error_c + i_m * acc_error_a
            if left_front == 0 or right_front == 0:
                change_speed = 20
            if left_middle == 0 and right_middle == 1:
                pwm_a += change_speed
                pwm_b += change_speed
                pwm_c += change_speed
                pwm_d += change_speed
            if right_middle == 0 and left_middle == 1:
                pwm_c -= change_speed
                pwm_d -= change_speed
                pwm_a -= change_speed
                pwm_b -= change_speed
            if left_rear == 0 or right_rear == 0:
                change_speed = 50
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)
        if error_a < -180:
            error_d = target_angle - now_angle + 360
            acc_error_a = acc_error_a + error_d
            pwm_a = p_b * error_d + i_b * acc_error_a
            pwm_b = p_b * error_d + i_b * acc_error_a
            pwm_c = p_m * error_d + i_m * acc_error_a
            pwm_d = p_m * error_d + i_m * acc_error_a
            if left_front == 0 or right_front == 0:
                change_speed = 20
            if left_middle == 0 and right_middle == 1:
                pwm_a += change_speed
                pwm_b += change_speed
                pwm_c += change_speed
                pwm_d += change_speed
            if right_middle == 0 and left_middle == 1:
                pwm_c -= change_speed
                pwm_d -= change_speed
                pwm_a -= change_speed
                pwm_b -= change_speed
            if left_rear == 0 or right_rear == 0:
                change_speed = 50
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)

        if right_front == 0 :
            front_condition += 1
        if right_middle == 0 :
            middle_condition += 1
        if front_condition > 0 and middle_condition > 0:
            bot.set_car_motion(0, 0, 0)
            # process_speed_back_road(20,"time.time()-start_time > 0.7")
            break
def process_speed_longroad_new_right_stop_back(set_speed):
    start_time = time.time()
    count = 0
    error_a = 0
    last_error_a = 0
    acc_error_a = 0
    front_condition = 0
    middle_condition = 0
    p_m = 2
    i_m = 0.06
    p_b = 2
    i_b = 0.06
    change_speed = 50

    left_front = get_GPIO(11)
    right_front = get_GPIO(5)
    left_rear = get_GPIO(19)
    right_rear = get_GPIO(26)
    left_middle = get_GPIO(9)
    right_middle = get_GPIO(6)
    pwm_a = pwm_b = pwm_c = pwm_d = 0
    a1 = b1 = c1 = d1 = set_speed

    useless = return_angle()
    target_angle = return_angle()
    bot.set_motor(set_speed, set_speed, set_speed, set_speed)
    while 1:
        left_front = get_GPIO(11)
        right_front = get_GPIO(5)
        left_rear = get_GPIO(19)
        right_rear = get_GPIO(26)
        left_middle = get_GPIO(9)
        right_middle = get_GPIO(6)
        crush_sensor = get_GPIO(13)
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
            if left_front == 0 or right_front == 0:
                change_speed = 20
            if left_middle == 0 and right_middle == 1:
                pwm_a += change_speed
                pwm_b += change_speed
                pwm_c += change_speed
                pwm_d += change_speed
            if right_middle == 0 and left_middle == 1:
                pwm_c -= change_speed
                pwm_d -= change_speed
                pwm_a -= change_speed
                pwm_b -= change_speed
            if left_rear == 0 or right_rear == 0:
                change_speed = 50
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)
        if error_a > 180:
            error_c = target_angle - now_angle - 360
            acc_error_a = acc_error_a + error_c
            pwm_a = p_b * error_c + i_b * acc_error_a
            pwm_b = p_b * error_c + i_b * acc_error_a
            pwm_c = p_m * error_c + i_m * acc_error_a
            pwm_d = p_m * error_c + i_m * acc_error_a
            if left_front == 0 or right_front == 0:
                change_speed = 20
            if left_middle == 0 and right_middle == 1:
                pwm_a += change_speed
                pwm_b += change_speed
                pwm_c += change_speed
                pwm_d += change_speed
            if right_middle == 0 and left_middle == 1:
                pwm_c -= change_speed
                pwm_d -= change_speed
                pwm_a -= change_speed
                pwm_b -= change_speed
            if left_rear == 0 or right_rear == 0:
                change_speed = 50
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)
        if error_a < -180:
            error_d = target_angle - now_angle + 360
            acc_error_a = acc_error_a + error_d
            pwm_a = p_b * error_d + i_b * acc_error_a
            pwm_b = p_b * error_d + i_b * acc_error_a
            pwm_c = p_m * error_d + i_m * acc_error_a
            pwm_d = p_m * error_d + i_m * acc_error_a
            if left_front == 0 or right_front == 0:
                change_speed = 20
            if left_middle == 0 and right_middle == 1:
                pwm_a += change_speed
                pwm_b += change_speed
                pwm_c += change_speed
                pwm_d += change_speed
            if right_middle == 0 and left_middle == 1:
                pwm_c -= change_speed
                pwm_d -= change_speed
                pwm_a -= change_speed
                pwm_b -= change_speed
            if left_rear == 0 or right_rear == 0:
                change_speed = 50
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)

        if right_front == 0 :
            front_condition += 1
        if right_middle == 0 :
            middle_condition += 1
        if front_condition > 0 and middle_condition > 0:
            bot.set_car_motion(0, 0, 0)
            process_speed_back_road(20,"time.time()-start_time > 0.7")
            break
def process_speed_longroad_new_right_addrear_stop_back(set_speed):
    start_time = time.time()
    count = 0
    error_a = 0
    last_error_a = 0
    acc_error_a = 0
    front_condition = 0
    middle_condition = 0
    rear_condition=0
    p_m = 2
    i_m = 0.06
    p_b = 2
    i_b = 0.06
    change_speed = 50

    left_front = get_GPIO(11)
    right_front = get_GPIO(5)
    left_rear = get_GPIO(19)
    right_rear = get_GPIO(26)
    left_middle = get_GPIO(9)
    right_middle = get_GPIO(6)
    pwm_a = pwm_b = pwm_c = pwm_d = 0
    a1 = b1 = c1 = d1 = set_speed

    useless = return_angle()
    target_angle = return_angle()
    bot.set_motor(set_speed, set_speed, set_speed, set_speed)
    while 1:
        left_front = get_GPIO(11)
        right_front = get_GPIO(5)
        left_rear = get_GPIO(19)
        right_rear = get_GPIO(26)
        left_middle = get_GPIO(9)
        right_middle = get_GPIO(6)
        crush_sensor = get_GPIO(13)
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
            if left_front == 0 or right_front == 0:
                change_speed = 20
            if left_middle == 0 and right_middle == 1:
                pwm_a += change_speed
                pwm_b += change_speed
                pwm_c += change_speed
                pwm_d += change_speed
            if right_middle == 0 and left_middle == 1:
                pwm_c -= change_speed
                pwm_d -= change_speed
                pwm_a -= change_speed
                pwm_b -= change_speed
            if left_rear == 0 or right_rear == 0:
                change_speed = 50
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)
        if error_a > 180:
            error_c = target_angle - now_angle - 360
            acc_error_a = acc_error_a + error_c
            pwm_a = p_b * error_c + i_b * acc_error_a
            pwm_b = p_b * error_c + i_b * acc_error_a
            pwm_c = p_m * error_c + i_m * acc_error_a
            pwm_d = p_m * error_c + i_m * acc_error_a
            if left_front == 0 or right_front == 0:
                change_speed = 20
            if left_middle == 0 and right_middle == 1:
                pwm_a += change_speed
                pwm_b += change_speed
                pwm_c += change_speed
                pwm_d += change_speed
            if right_middle == 0 and left_middle == 1:
                pwm_c -= change_speed
                pwm_d -= change_speed
                pwm_a -= change_speed
                pwm_b -= change_speed
            if left_rear == 0 or right_rear == 0:
                change_speed = 50
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)
        if error_a < -180:
            error_d = target_angle - now_angle + 360
            acc_error_a = acc_error_a + error_d
            pwm_a = p_b * error_d + i_b * acc_error_a
            pwm_b = p_b * error_d + i_b * acc_error_a
            pwm_c = p_m * error_d + i_m * acc_error_a
            pwm_d = p_m * error_d + i_m * acc_error_a
            if left_front == 0 or right_front == 0:
                change_speed = 20
            if left_middle == 0 and right_middle == 1:
                pwm_a += change_speed
                pwm_b += change_speed
                pwm_c += change_speed
                pwm_d += change_speed
            if right_middle == 0 and left_middle == 1:
                pwm_c -= change_speed
                pwm_d -= change_speed
                pwm_a -= change_speed
                pwm_b -= change_speed
            if left_rear == 0 or right_rear == 0:
                change_speed = 50
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)

        if right_front == 0 :
            front_condition += 1
        if right_middle == 0 :
            middle_condition += 1
        if right_rear ==0:
            rear_condition+=1
        if front_condition > 0 and middle_condition > 0 and rear_condition>0:
            bot.set_car_motion(0, 0, 0)
            process_speed_back_road(20,"time.time()-start_time > 0.7")
            break
def process_speed_longroad_new_rightaddleft_launch(set_speed):
    start_time = time.time()
    count = 0
    error_a = 0
    last_error_a = 0
    acc_error_a = 0
    front_condition = 0
    middle_condition = 0
    rear_condition=0
    p_m = 2
    i_m = 0.06
    p_b = 2
    i_b = 0.06
    change_speed = 50

    left_front = get_GPIO(11)
    right_front = get_GPIO(5)
    left_rear = get_GPIO(19)
    right_rear = get_GPIO(26)
    left_middle = get_GPIO(9)
    right_middle = get_GPIO(6)
    pwm_a = pwm_b = pwm_c = pwm_d = 0
    a1 = b1 = c1 = d1 = set_speed

    useless = return_angle()
    target_angle = return_angle()
    bot.set_motor(set_speed, set_speed, set_speed, set_speed)
    while 1:
        left_front = get_GPIO(11)
        right_front = get_GPIO(5)
        left_rear = get_GPIO(19)
        right_rear = get_GPIO(26)
        left_middle = get_GPIO(9)
        right_middle = get_GPIO(6)
        crush_sensor = get_GPIO(13)
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
            if left_front == 0 or right_front == 0:
                change_speed = 20
            if left_middle == 0 and right_middle == 1:
                pwm_a += change_speed
                pwm_b += change_speed
                pwm_c += change_speed
                pwm_d += change_speed
            if right_middle == 0 and left_middle == 1:
                pwm_c -= change_speed
                pwm_d -= change_speed
                pwm_a -= change_speed
                pwm_b -= change_speed
            if left_rear == 0 or right_rear == 0:
                change_speed = 50
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)
        if error_a > 180:
            error_c = target_angle - now_angle - 360
            acc_error_a = acc_error_a + error_c
            pwm_a = p_b * error_c + i_b * acc_error_a
            pwm_b = p_b * error_c + i_b * acc_error_a
            pwm_c = p_m * error_c + i_m * acc_error_a
            pwm_d = p_m * error_c + i_m * acc_error_a
            if left_front == 0 or right_front == 0:
                change_speed = 20
            if left_middle == 0 and right_middle == 1:
                pwm_a += change_speed
                pwm_b += change_speed
                pwm_c += change_speed
                pwm_d += change_speed
            if right_middle == 0 and left_middle == 1:
                pwm_c -= change_speed
                pwm_d -= change_speed
                pwm_a -= change_speed
                pwm_b -= change_speed
            if left_rear == 0 or right_rear == 0:
                change_speed = 50
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)
        if error_a < -180:
            error_d = target_angle - now_angle + 360
            acc_error_a = acc_error_a + error_d
            pwm_a = p_b * error_d + i_b * acc_error_a
            pwm_b = p_b * error_d + i_b * acc_error_a
            pwm_c = p_m * error_d + i_m * acc_error_a
            pwm_d = p_m * error_d + i_m * acc_error_a
            if left_front == 0 or right_front == 0:
                change_speed = 20
            if left_middle == 0 and right_middle == 1:
                pwm_a += change_speed
                pwm_b += change_speed
                pwm_c += change_speed
                pwm_d += change_speed
            if right_middle == 0 and left_middle == 1:
                pwm_c -= change_speed
                pwm_d -= change_speed
                pwm_a -= change_speed
                pwm_b -= change_speed
            if left_rear == 0 or right_rear == 0:
                change_speed = 50
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)

        if right_front == 0 and left_front==0:
            front_condition += 1
        if right_middle == 0 and left_middle==0:
            middle_condition += 1
        #if right_rear ==0:
           # rear_condition+=1
        if front_condition > 0 and middle_condition > 0 :
            #bot.set_car_motion(0, 0, 0)
            #process_speed_back_road(20,"time.time()-start_time > 0.7")
            break
def process_speed_longroad_new_left_stop(set_speed):
    start_time = time.time()
    count = 0
    error_a = 0
    last_error_a = 0
    acc_error_a = 0
    front_condition = 0
    middle_condition = 0
    p_m = 2
    i_m = 0.06
    p_b = 2
    i_b = 0.06
    change_speed = 50

    left_front = get_GPIO(11)
    right_front = get_GPIO(5)
    left_rear = get_GPIO(19)
    right_rear = get_GPIO(26)
    left_middle = get_GPIO(9)
    right_middle = get_GPIO(6)
    pwm_a = pwm_b = pwm_c = pwm_d = 0
    a1 = b1 = c1 = d1 = set_speed

    useless = return_angle()
    target_angle = return_angle()
    bot.set_motor(set_speed, set_speed, set_speed, set_speed)
    while 1:
        left_front = get_GPIO(11)
        right_front = get_GPIO(5)
        left_rear = get_GPIO(19)
        right_rear = get_GPIO(26)
        left_middle = get_GPIO(9)
        right_middle = get_GPIO(6)
        crush_sensor = get_GPIO(13)
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
            if left_front == 0 or right_front == 0:
                change_speed = 20
            if left_middle == 0 and right_middle == 1:
                pwm_a += change_speed
                pwm_b += change_speed
                pwm_c += change_speed
                pwm_d += change_speed
            if right_middle == 0 and left_middle == 1:
                pwm_c -= change_speed
                pwm_d -= change_speed
                pwm_a -= change_speed
                pwm_b -= change_speed
            if left_rear == 0 or right_rear == 0:
                change_speed = 50
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)
        if error_a > 180:
            error_c = target_angle - now_angle - 360
            acc_error_a = acc_error_a + error_c
            pwm_a = p_b * error_c + i_b * acc_error_a
            pwm_b = p_b * error_c + i_b * acc_error_a
            pwm_c = p_m * error_c + i_m * acc_error_a
            pwm_d = p_m * error_c + i_m * acc_error_a
            if left_front == 0 or right_front == 0:
                change_speed = 20
            if left_middle == 0 and right_middle == 1:
                pwm_a += change_speed
                pwm_b += change_speed
                pwm_c += change_speed
                pwm_d += change_speed
            if right_middle == 0 and left_middle == 1:
                pwm_c -= change_speed
                pwm_d -= change_speed
                pwm_a -= change_speed
                pwm_b -= change_speed
            if left_rear == 0 or right_rear == 0:
                change_speed = 50
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)
        if error_a < -180:
            error_d = target_angle - now_angle + 360
            acc_error_a = acc_error_a + error_d
            pwm_a = p_b * error_d + i_b * acc_error_a
            pwm_b = p_b * error_d + i_b * acc_error_a
            pwm_c = p_m * error_d + i_m * acc_error_a
            pwm_d = p_m * error_d + i_m * acc_error_a
            if left_front == 0 or right_front == 0:
                change_speed = 20
            if left_middle == 0 and right_middle == 1:
                pwm_a += change_speed
                pwm_b += change_speed
                pwm_c += change_speed
                pwm_d += change_speed
            if right_middle == 0 and left_middle == 1:
                pwm_c -= change_speed
                pwm_d -= change_speed
                pwm_a -= change_speed
                pwm_b -= change_speed
            if left_rear == 0 or right_rear == 0:
                change_speed = 50
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)

        if left_front == 0 :
            front_condition += 1
        if left_middle == 0 :
            middle_condition += 1
        if front_condition > 0 and middle_condition > 0:
            bot.set_car_motion(0, 0, 0)
            # process_speed_back_road(20,"time.time()-start_time > 0.7")
            break
def process_speed_longroad_new_left_stop_back(set_speed):
    start_time = time.time()
    count = 0
    error_a = 0
    last_error_a = 0
    acc_error_a = 0
    front_condition = 0
    middle_condition = 0
    p_m = 2
    i_m = 0.06
    p_b = 2
    i_b = 0.06
    change_speed = 50

    left_front = get_GPIO(11)
    right_front = get_GPIO(5)
    left_rear = get_GPIO(19)
    right_rear = get_GPIO(26)
    left_middle = get_GPIO(9)
    right_middle = get_GPIO(6)
    pwm_a = pwm_b = pwm_c = pwm_d = 0
    a1 = b1 = c1 = d1 = set_speed

    useless = return_angle()
    target_angle = return_angle()
    bot.set_motor(set_speed, set_speed, set_speed, set_speed)
    while 1:
        left_front = get_GPIO(11)
        right_front = get_GPIO(5)
        left_rear = get_GPIO(19)
        right_rear = get_GPIO(26)
        left_middle = get_GPIO(9)
        right_middle = get_GPIO(6)
        crush_sensor = get_GPIO(13)
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
            if left_front == 0 or right_front == 0:
                change_speed = 20
            if left_middle == 0 and right_middle == 1:
                pwm_a += change_speed
                pwm_b += change_speed
                pwm_c += change_speed
                pwm_d += change_speed
            if right_middle == 0 and left_middle == 1:
                pwm_c -= change_speed
                pwm_d -= change_speed
                pwm_a -= change_speed
                pwm_b -= change_speed
            if left_rear == 0 or right_rear == 0:
                change_speed = 50
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)
        if error_a > 180:
            error_c = target_angle - now_angle - 360
            acc_error_a = acc_error_a + error_c
            pwm_a = p_b * error_c + i_b * acc_error_a
            pwm_b = p_b * error_c + i_b * acc_error_a
            pwm_c = p_m * error_c + i_m * acc_error_a
            pwm_d = p_m * error_c + i_m * acc_error_a
            if left_front == 0 or right_front == 0:
                change_speed = 20
            if left_middle == 0 and right_middle == 1:
                pwm_a += change_speed
                pwm_b += change_speed
                pwm_c += change_speed
                pwm_d += change_speed
            if right_middle == 0 and left_middle == 1:
                pwm_c -= change_speed
                pwm_d -= change_speed
                pwm_a -= change_speed
                pwm_b -= change_speed
            if left_rear == 0 or right_rear == 0:
                change_speed = 50
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)
        if error_a < -180:
            error_d = target_angle - now_angle + 360
            acc_error_a = acc_error_a + error_d
            pwm_a = p_b * error_d + i_b * acc_error_a
            pwm_b = p_b * error_d + i_b * acc_error_a
            pwm_c = p_m * error_d + i_m * acc_error_a
            pwm_d = p_m * error_d + i_m * acc_error_a
            if left_front == 0 or right_front == 0:
                change_speed = 20
            if left_middle == 0 and right_middle == 1:
                pwm_a += change_speed
                pwm_b += change_speed
                pwm_c += change_speed
                pwm_d += change_speed
            if right_middle == 0 and left_middle == 1:
                pwm_c -= change_speed
                pwm_d -= change_speed
                pwm_a -= change_speed
                pwm_b -= change_speed
            if left_rear == 0 or right_rear == 0:
                change_speed = 50
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)

        if left_front == 0 :
            front_condition += 1
        if left_middle == 0 :
            middle_condition += 1
        if front_condition > 0 and middle_condition > 0:
            bot.set_car_motion(0, 0, 0)
            process_speed_back_road(20,"time.time()-start_time > 0.7")
            break
def process_speed_longroad_new_414(set_speed):
    start_time = time.time()
    count = 0
    error_a = 0
    last_error_a = 0
    acc_error_a = 0
    front_condition = 0
    middle_condition = 0
    p_m = 2
    i_m = 0.06
    p_b = 2
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
    
    useless = return_angle()
    target_angle = return_angle()
    bot.set_motor(set_speed, set_speed, set_speed, set_speed)
    while 1:
        left_front = get_GPIO(11)
        right_front = get_GPIO(5)
        left_rear = get_GPIO(19)
        right_rear = get_GPIO(26)
        left_middle = get_GPIO(9)
        right_middle = get_GPIO(6)
        crush_sensor = get_GPIO(13)
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
            if left_front==0 or right_front==0:
                change_speed=20
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
            if left_rear==0 or right_rear==0:
                change_speed=50
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)
        if error_a > 180:
            error_c = target_angle - now_angle - 360
            acc_error_a = acc_error_a + error_c
            pwm_a = p_b * error_c + i_b * acc_error_a
            pwm_b = p_b * error_c + i_b * acc_error_a
            pwm_c = p_m * error_c + i_m * acc_error_a
            pwm_d = p_m * error_c + i_m * acc_error_a
            if left_front==0 or right_front==0:
                change_speed=20
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
            if left_rear==0 or right_rear==0:
                change_speed=50
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)
        if error_a < -180:
            error_d = target_angle - now_angle + 360
            acc_error_a = acc_error_a + error_d
            pwm_a = p_b * error_d + i_b * acc_error_a
            pwm_b = p_b * error_d + i_b * acc_error_a
            pwm_c = p_m * error_d + i_m * acc_error_a
            pwm_d = p_m * error_d + i_m * acc_error_a
            if left_front==0 or right_front==0:
                change_speed=20
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
            if left_rear==0 or right_rear==0:
                change_speed=50
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)

        if left_front==0 or right_front==0:
            front_condition +=1
       
        if front_condition > 0 :
            bot.set_car_motion(0,0,0)
            break
def process_speed_longroad_new_415(set_speed):
    start_time = time.time()
    count = 0
    error_a = 0
    last_error_a = 0
    acc_error_a = 0
    front_condition = 0
    middle_condition = 0
    p_m = 2
    i_m = 0.06
    p_b = 2
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
    
    useless = return_angle()
    target_angle = return_angle()
    bot.set_motor(set_speed, set_speed, set_speed, set_speed)
    while 1:
        left_front = get_GPIO(11)
        right_front = get_GPIO(5)
        left_rear = get_GPIO(19)
        right_rear = get_GPIO(26)
        left_middle = get_GPIO(9)
        right_middle = get_GPIO(6)
        crush_sensor = get_GPIO(13)
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
            if left_front==0 or right_front==0:
                change_speed=20
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
            if left_rear==0 or right_rear==0:
                change_speed=50
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)
        if error_a > 180:
            error_c = target_angle - now_angle - 360
            acc_error_a = acc_error_a + error_c
            pwm_a = p_b * error_c + i_b * acc_error_a
            pwm_b = p_b * error_c + i_b * acc_error_a
            pwm_c = p_m * error_c + i_m * acc_error_a
            pwm_d = p_m * error_c + i_m * acc_error_a
            if left_front==0 or right_front==0:
                change_speed=20
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
            if left_rear==0 or right_rear==0:
                change_speed=50
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)
        if error_a < -180:
            error_d = target_angle - now_angle + 360
            acc_error_a = acc_error_a + error_d
            pwm_a = p_b * error_d + i_b * acc_error_a
            pwm_b = p_b * error_d + i_b * acc_error_a
            pwm_c = p_m * error_d + i_m * acc_error_a
            pwm_d = p_m * error_d + i_m * acc_error_a
            if left_front==0 or right_front==0:
                change_speed=20
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
            if left_rear==0 or right_rear==0:
                change_speed=50
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)

        if  right_front==0:
            front_condition +=1
        if right_middle==0:
            middle_condition+=1
        if front_condition > 0 :
            bot.set_car_motion(0,0,0)
            break
def process_speed_longroad_new_418(set_speed):
    start_time = time.time()
    count = 0
    error_a = 0
    last_error_a = 0
    acc_error_a = 0
    front_condition = 0
    middle_condition = 0
    p_m = 2
    i_m = 0.06
    p_b = 2
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
    
    useless = return_angle()
    target_angle = return_angle()
    bot.set_motor(set_speed, set_speed, set_speed, set_speed)
    while 1:
        left_front = get_GPIO(11)
        right_front = get_GPIO(5)
        left_rear = get_GPIO(19)
        right_rear = get_GPIO(26)
        left_middle = get_GPIO(9)
        right_middle = get_GPIO(6)
        crush_sensor = get_GPIO(13)
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
            if left_front==0 or right_front==0:
                change_speed=20
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
            if left_rear==0 or right_rear==0:
                change_speed=50
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)
        if error_a > 180:
            error_c = target_angle - now_angle - 360
            acc_error_a = acc_error_a + error_c
            pwm_a = p_b * error_c + i_b * acc_error_a
            pwm_b = p_b * error_c + i_b * acc_error_a
            pwm_c = p_m * error_c + i_m * acc_error_a
            pwm_d = p_m * error_c + i_m * acc_error_a
            if left_front==0 or right_front==0:
                change_speed=20
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
            if left_rear==0 or right_rear==0:
                change_speed=50
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)
        if error_a < -180:
            error_d = target_angle - now_angle + 360
            acc_error_a = acc_error_a + error_d
            pwm_a = p_b * error_d + i_b * acc_error_a
            pwm_b = p_b * error_d + i_b * acc_error_a
            pwm_c = p_m * error_d + i_m * acc_error_a
            pwm_d = p_m * error_d + i_m * acc_error_a
            if left_front==0 or right_front==0:
                change_speed=20
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
            if left_rear==0 or right_rear==0:
                change_speed=50
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)

        if  left_front==0:
            front_condition +=1
        if left_middle==0:
            middle_condition+=1
        if front_condition > 0 and middle_condition >0:
            bot.set_car_motion(0,0,0)
            break
def process_speed_longroad_new_510(set_speed):
    start_time = time.time()
    count = 0
    error_a = 0
    last_error_a = 0
    acc_error_a = 0
    front_condition = 0
    middle_condition = 0
    rear_condition=0
    p_m = 2
    i_m = 0.06
    p_b = 2
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
    
    useless = return_angle()
    target_angle = return_angle()
    bot.set_motor(set_speed, set_speed, set_speed, set_speed)
    while 1:
        left_front = get_GPIO(11)
        right_front = get_GPIO(5)
        left_rear = get_GPIO(19)
        right_rear = get_GPIO(26)
        left_middle = get_GPIO(9)
        right_middle = get_GPIO(6)
        crush_sensor = get_GPIO(13)
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
            if left_front==0 or right_front==0:
                change_speed=20
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
            if left_rear==0 or right_rear==0:
                change_speed=50
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)
        if error_a > 180:
            error_c = target_angle - now_angle - 360
            acc_error_a = acc_error_a + error_c
            pwm_a = p_b * error_c + i_b * acc_error_a
            pwm_b = p_b * error_c + i_b * acc_error_a
            pwm_c = p_m * error_c + i_m * acc_error_a
            pwm_d = p_m * error_c + i_m * acc_error_a
            if left_front==0 or right_front==0:
                change_speed=20
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
            if left_rear==0 or right_rear==0:
                change_speed=50
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)
        if error_a < -180:
            error_d = target_angle - now_angle + 360
            acc_error_a = acc_error_a + error_d
            pwm_a = p_b * error_d + i_b * acc_error_a
            pwm_b = p_b * error_d + i_b * acc_error_a
            pwm_c = p_m * error_d + i_m * acc_error_a
            pwm_d = p_m * error_d + i_m * acc_error_a
            if left_front==0 or right_front==0:
                change_speed=20
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
            if left_rear==0 or right_rear==0:
                change_speed=50
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)

        if  left_front==0 :
            front_condition +=1
        if left_middle==0 :
                middle_condition+=1
        if left_rear==0:
                rear_condition+=1
        if left_rear==0 and left_middle==0 and left_front==0:
                middle_condition=0
                rear_condition=0
                front_condition=0
        if front_condition  > 0 and middle_condition >0 and rear_condition >0:
            bot.set_car_motion(0,0,0)
            process_speed_back_road(-20,"time.time()-start_time > 0.5")  #
            break
def process_speed_longroad_new_419(set_speed):
    start_time = time.time()
    count = 0
    error_a = 0
    last_error_a = 0
    acc_error_a = 0
    front_condition_right = 0
    front_condition_left =0
    middle_condition = 0
    p_m = 2
    i_m = 0.06
    p_b = 2
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
    
    useless = return_angle()
    target_angle = return_angle()
    bot.set_motor(set_speed, set_speed, set_speed, set_speed)
    while 1:
        left_front = get_GPIO(11)
        right_front = get_GPIO(5)
        left_rear = get_GPIO(19)
        right_rear = get_GPIO(26)
        left_middle = get_GPIO(9)
        right_middle = get_GPIO(6)
        crush_sensor = get_GPIO(13)
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
            if left_front==0 or right_front==0:
                change_speed=20
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
            if left_rear==0 or right_rear==0:
                change_speed=50
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)
        if error_a > 180:
            error_c = target_angle - now_angle - 360
            acc_error_a = acc_error_a + error_c
            pwm_a = p_b * error_c + i_b * acc_error_a
            pwm_b = p_b * error_c + i_b * acc_error_a
            pwm_c = p_m * error_c + i_m * acc_error_a
            pwm_d = p_m * error_c + i_m * acc_error_a
            if left_front==0 or right_front==0:
                change_speed=20
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
            if left_rear==0 or right_rear==0:
                change_speed=50
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)
        if error_a < -180:
            error_d = target_angle - now_angle + 360
            acc_error_a = acc_error_a + error_d
            pwm_a = p_b * error_d + i_b * acc_error_a
            pwm_b = p_b * error_d + i_b * acc_error_a
            pwm_c = p_m * error_d + i_m * acc_error_a
            pwm_d = p_m * error_d + i_m * acc_error_a
            if left_front==0 or right_front==0:
                change_speed=20
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
            if left_rear==0 or right_rear==0:
                change_speed=50
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)

        if  left_front==0:
            front_condition_right +=1
        if right_front==0:
            front_condition_left+=1
        if front_condition_left > 0 and front_condition_right > 0:
            bot.set_car_motion(0,0,0)
            break


def process_speed_longroad_new_10(set_speed):
    #back
    start_time = time.time()
    count = 0
    error_a = 0
    last_error_a = 0
    acc_error_a = 0
    front_condition = 0
    middle_condition = 0
    rear_condition=0
    p_m = 2
    i_m = 0.06
    p_b = 2
    i_b = 0.06
    change_speed = 50

    left_front = get_GPIO(11)
    right_front = get_GPIO(5)
    left_rear = get_GPIO(19)
    right_rear = get_GPIO(26)
    left_middle = get_GPIO(9)
    right_middle = get_GPIO(6)
    pwm_a = pwm_b = pwm_c = pwm_d = 0
    a1 = b1 = c1 = d1 = set_speed

    useless = return_angle()
    target_angle = return_angle()
    bot.set_motor(set_speed, set_speed, set_speed, set_speed)
    while 1:
        left_front = get_GPIO(11)
        right_front = get_GPIO(5)
        left_rear = get_GPIO(19)
        right_rear = get_GPIO(26)
        left_middle = get_GPIO(9)
        right_middle = get_GPIO(6)
        crush_sensor = get_GPIO(13)
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
            if left_front == 0 or right_front == 0:
                change_speed = 20
            if left_middle == 0 and right_middle == 1:
                pwm_a += change_speed
                pwm_b += change_speed
                pwm_c += change_speed
                pwm_d += change_speed
            if right_middle == 0 and left_middle == 1:
                pwm_c -= change_speed
                pwm_d -= change_speed
                pwm_a -= change_speed
                pwm_b -= change_speed

            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)
        if error_a > 180:
            error_c = target_angle - now_angle - 360
            acc_error_a = acc_error_a + error_c
            pwm_a = p_b * error_c + i_b * acc_error_a
            pwm_b = p_b * error_c + i_b * acc_error_a
            pwm_c = p_m * error_c + i_m * acc_error_a
            pwm_d = p_m * error_c + i_m * acc_error_a
            if left_front == 0 or right_front == 0:
                change_speed = 20
            if left_middle == 0 and right_middle == 1:
                pwm_a += change_speed
                pwm_b += change_speed
                pwm_c += change_speed
                pwm_d += change_speed
            if right_middle == 0 and left_middle == 1:
                pwm_c -= change_speed
                pwm_d -= change_speed
                pwm_a -= change_speed
                pwm_b -= change_speed

            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)
        if error_a < -180:
            error_d = target_angle - now_angle + 360
            acc_error_a = acc_error_a + error_d
            pwm_a = p_b * error_d + i_b * acc_error_a
            pwm_b = p_b * error_d + i_b * acc_error_a
            pwm_c = p_m * error_d + i_m * acc_error_a
            pwm_d = p_m * error_d + i_m * acc_error_a
            if left_front == 0 or right_front == 0:
                change_speed = 20
            if left_middle == 0 and right_middle == 1:
                pwm_a += change_speed
                pwm_b += change_speed
                pwm_c += change_speed
                pwm_d += change_speed
            if right_middle == 0 and left_middle == 1:
                pwm_c -= change_speed
                pwm_d -= change_speed
                pwm_a -= change_speed
                pwm_b -= change_speed

            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)

        if left_front == 0 :
            front_condition += 1
        if left_middle == 0 or right_middle == 0:
            middle_condition += 1
        if left_rear ==0:
            rear_condition +=1
        if front_condition > 0 and middle_condition > 0:
            bot.set_car_motion(0, 0, 0)
            process_speed_back_road(-20, "time.time()-start_time > 0.7")
            break

        # if eval(exit_condition):   # 退出条件
        # bot.set_motor(0, 0, 0, 0)
        # break
def process_speed_longroad_new(set_speed):
    start_time = time.time()
    count = 0
    error_a = 0
    last_error_a = 0
    acc_error_a = 0
    front_condition = 0
    middle_condition = 0
    p_m = 2
    i_m = 0.06
    p_b = 2
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
    
    useless = return_angle()
    target_angle = return_angle()
    bot.set_motor(set_speed, set_speed, set_speed, set_speed)
    while 1:
        left_front = get_GPIO(11)
        right_front = get_GPIO(5)
        left_rear = get_GPIO(19)
        right_rear = get_GPIO(26)
        left_middle = get_GPIO(9)
        right_middle = get_GPIO(6)
        crush_sensor = get_GPIO(13)
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
            if left_front==0 or right_front==0:
                change_speed=20
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
            if left_front==0 or right_front==0:
                change_speed=20
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
            if left_front==0 or right_front==0:
                change_speed=20
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

        if left_front==0 or right_front==0:
            front_condition +=1
        if left_middle==0 or right_middle==0:
            middle_condition +=1
        if front_condition > 0 and middle_condition >0:
            bot.set_car_motion(0,0,0)
            break

        #if eval(exit_condition):   # 退出条件
            #bot.set_motor(0, 0, 0, 0)
            #break


        #if eval(exit_condition):   # 退出条件
            #bot.set_motor(0, 0, 0, 0)
            #break


def process_speed_longroad_new_420(set_speed):
    start_time = time.time()
    count = 0
    error_a = 0
    last_error_a = 0
    acc_error_a = 0
    front_condition = 0
    middle_condition = 0
    p_m = 2
    i_m = 0.06
    p_b = 2
    i_b = 0.06
    change_speed = 50

    left_front = get_GPIO(11)
    right_front = get_GPIO(5)
    left_rear = get_GPIO(19)
    right_rear = get_GPIO(26)
    left_middle = get_GPIO(9)
    right_middle = get_GPIO(6)
    pwm_a = pwm_b = pwm_c = pwm_d = 0
    a1 = b1 = c1 = d1 = set_speed

    useless = return_angle()
    target_angle = return_angle()
    bot.set_motor(set_speed, set_speed, set_speed, set_speed)
    while 1:
        left_front = get_GPIO(11)
        right_front = get_GPIO(5)
        left_rear = get_GPIO(19)
        right_rear = get_GPIO(26)
        left_middle = get_GPIO(9)
        right_middle = get_GPIO(6)
        crush_sensor = get_GPIO(13)
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
            if left_front == 0 or right_front == 0:
                change_speed = 20
            if left_middle == 0 and right_middle == 1:
                pwm_a += change_speed
                pwm_b += change_speed
                pwm_c += change_speed
                pwm_d += change_speed
            if right_middle == 0 and left_middle == 1:
                pwm_c -= change_speed
                pwm_d -= change_speed
                pwm_a -= change_speed
                pwm_b -= change_speed

            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)
        if error_a > 180:
            error_c = target_angle - now_angle - 360
            acc_error_a = acc_error_a + error_c
            pwm_a = p_b * error_c + i_b * acc_error_a
            pwm_b = p_b * error_c + i_b * acc_error_a
            pwm_c = p_m * error_c + i_m * acc_error_a
            pwm_d = p_m * error_c + i_m * acc_error_a
            if left_front == 0 or right_front == 0:
                change_speed = 20
            if left_middle == 0 and right_middle == 1:
                pwm_a += change_speed
                pwm_b += change_speed
                pwm_c += change_speed
                pwm_d += change_speed
            if right_middle == 0 and left_middle == 1:
                pwm_c -= change_speed
                pwm_d -= change_speed
                pwm_a -= change_speed
                pwm_b -= change_speed

            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)
        if error_a < -180:
            error_d = target_angle - now_angle + 360
            acc_error_a = acc_error_a + error_d
            pwm_a = p_b * error_d + i_b * acc_error_a
            pwm_b = p_b * error_d + i_b * acc_error_a
            pwm_c = p_m * error_d + i_m * acc_error_a
            pwm_d = p_m * error_d + i_m * acc_error_a
            if left_front == 0 or right_front == 0:
                change_speed = 20
            if left_middle == 0 and right_middle == 1:
                pwm_a += change_speed
                pwm_b += change_speed
                pwm_c += change_speed
                pwm_d += change_speed
            if right_middle == 0 and left_middle == 1:
                pwm_c -= change_speed
                pwm_d -= change_speed
                pwm_a -= change_speed
                pwm_b -= change_speed

            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)

        if left_front == 0 or right_front == 0:
            front_condition += 1
        if left_middle == 0 or right_middle == 0:
            middle_condition += 1
        if front_condition > 0 and middle_condition > 0:
            break
def process_speed_longroad_new_light(set_speed):
    start_time = time.time()
    count = 0
    error_a = 0
    last_error_a = 0
    acc_error_a = 0
    front_condition = 0
    middle_condition = 0
    p_m = 2
    i_m = 0.06
    p_b = 2
    i_b = 0.06
    change_speed = 50

    left_front = get_GPIO(11)
    right_front = get_GPIO(5)
    left_rear = get_GPIO(19)
    right_rear = get_GPIO(26)
    left_middle = get_GPIO(9)
    right_middle = get_GPIO(6)
    pwm_a = pwm_b = pwm_c = pwm_d = 0
    a1 = b1 = c1 = d1 = set_speed

    useless = return_angle()
    target_angle = return_angle()
    bot.set_motor(set_speed, set_speed, set_speed, set_speed)
    while 1:
        left_front = get_GPIO(11)
        right_front = get_GPIO(5)
        left_rear = get_GPIO(19)
        right_rear = get_GPIO(26)
        left_middle = get_GPIO(9)
        right_middle = get_GPIO(6)
        crush_sensor = get_GPIO(13)
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
            if left_front == 0 or right_front == 0:
                change_speed = 20
            if left_middle == 0 and right_middle == 1:
                pwm_a += change_speed
                pwm_b += change_speed
                pwm_c += change_speed
                pwm_d += change_speed
            if right_middle == 0 and left_middle == 1:
                pwm_c -= change_speed
                pwm_d -= change_speed
                pwm_a -= change_speed
                pwm_b -= change_speed

            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)
        if error_a > 180:
            error_c = target_angle - now_angle - 360
            acc_error_a = acc_error_a + error_c
            pwm_a = p_b * error_c + i_b * acc_error_a
            pwm_b = p_b * error_c + i_b * acc_error_a
            pwm_c = p_m * error_c + i_m * acc_error_a
            pwm_d = p_m * error_c + i_m * acc_error_a
            if left_front == 0 or right_front == 0:
                change_speed = 20
            if left_middle == 0 and right_middle == 1:
                pwm_a += change_speed
                pwm_b += change_speed
                pwm_c += change_speed
                pwm_d += change_speed
            if right_middle == 0 and left_middle == 1:
                pwm_c -= change_speed
                pwm_d -= change_speed
                pwm_a -= change_speed
                pwm_b -= change_speed

            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)
        if error_a < -180:
            error_d = target_angle - now_angle + 360
            acc_error_a = acc_error_a + error_d
            pwm_a = p_b * error_d + i_b * acc_error_a
            pwm_b = p_b * error_d + i_b * acc_error_a
            pwm_c = p_m * error_d + i_m * acc_error_a
            pwm_d = p_m * error_d + i_m * acc_error_a
            if left_front == 0 or right_front == 0:
                change_speed = 20
            if left_middle == 0 and right_middle == 1:
                pwm_a += change_speed
                pwm_b += change_speed
                pwm_c += change_speed
                pwm_d += change_speed
            if right_middle == 0 and left_middle == 1:
                pwm_c -= change_speed
                pwm_d -= change_speed
                pwm_a -= change_speed
                pwm_b -= change_speed

            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)

        if left_front == 0 or right_front == 0:
            front_condition += 1
        if left_middle == 0 or right_middle == 0:
            middle_condition += 1
        if front_condition > 0 and middle_condition > 0:
            bot.set_car_motion(0, 0, 0)
            process_speed_back_road(-20, "time.time()-start_time > 0.7")
            break
def process_speed_longroad_new_11(set_speed):
    start_time = time.time()
    count = 0
    error_a = 0
    last_error_a = 0
    acc_error_a = 0
    front_condition = 0
    middle_condition = 0
    rear_condition =0
    p_m = 2
    i_m = 0.06
    p_b = 2
    i_b = 0.06
    change_speed = 50

    left_front = get_GPIO(11)
    right_front = get_GPIO(5)
    left_rear = get_GPIO(19)
    right_rear = get_GPIO(26)
    left_middle = get_GPIO(9)
    right_middle = get_GPIO(6)
    pwm_a = pwm_b = pwm_c = pwm_d = 0
    a1 = b1 = c1 = d1 = set_speed

    useless = return_angle()
    target_angle = return_angle()
    bot.set_motor(set_speed, set_speed, set_speed, set_speed)
    while 1:
        left_front = get_GPIO(11)
        right_front = get_GPIO(5)
        left_rear = get_GPIO(19)
        right_rear = get_GPIO(26)
        left_middle = get_GPIO(9)
        right_middle = get_GPIO(6)
        crush_sensor = get_GPIO(13)
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
            if left_front == 0 or right_front == 0:
                change_speed = 20
            if left_middle == 0 and right_middle == 1:
                pwm_a += change_speed
                pwm_b += change_speed
                pwm_c += change_speed
                pwm_d += change_speed
            if right_middle == 0 and left_middle == 1:
                pwm_c -= change_speed
                pwm_d -= change_speed
                pwm_a -= change_speed
                pwm_b -= change_speed

            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)
        if error_a > 180:
            error_c = target_angle - now_angle - 360
            acc_error_a = acc_error_a + error_c
            pwm_a = p_b * error_c + i_b * acc_error_a
            pwm_b = p_b * error_c + i_b * acc_error_a
            pwm_c = p_m * error_c + i_m * acc_error_a
            pwm_d = p_m * error_c + i_m * acc_error_a
            if left_front == 0 or right_front == 0:
                change_speed = 20
            if left_middle == 0 and right_middle == 1:
                pwm_a += change_speed
                pwm_b += change_speed
                pwm_c += change_speed
                pwm_d += change_speed
            if right_middle == 0 and left_middle == 1:
                pwm_c -= change_speed
                pwm_d -= change_speed
                pwm_a -= change_speed
                pwm_b -= change_speed

            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)
        if error_a < -180:
            error_d = target_angle - now_angle + 360
            acc_error_a = acc_error_a + error_d
            pwm_a = p_b * error_d + i_b * acc_error_a
            pwm_b = p_b * error_d + i_b * acc_error_a
            pwm_c = p_m * error_d + i_m * acc_error_a
            pwm_d = p_m * error_d + i_m * acc_error_a
            if left_front == 0 or right_front == 0:
                change_speed = 20
            if left_middle == 0 and right_middle == 1:
                pwm_a += change_speed
                pwm_b += change_speed
                pwm_c += change_speed
                pwm_d += change_speed
            if right_middle == 0 and left_middle == 1:
                pwm_c -= change_speed
                pwm_d -= change_speed
                pwm_a -= change_speed
                pwm_b -= change_speed

            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)

        if left_front == 0 or right_front == 0:
            front_condition += 1
        if left_middle == 0 or right_middle == 0:
            middle_condition += 1
        if right_rear ==0 :
            rear_condition+=1
        if front_condition > 0 and middle_condition > 0 and rear_condition >0:
            bot.set_car_motion(0, 0, 0)
            process_speed_back_road(-20,"time.time()-start_time > 0.7")
            break
def process_speed_longroad_new_417(set_speed):
    start_time = time.time()
    count = 0
    error_a = 0
    last_error_a = 0
    acc_error_a = 0
    front_condition = 0
    middle_condition = 0
    p_m = 2
    i_m = 0.06
    p_b = 2
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
    
    useless = return_angle()
    target_angle = return_angle()
    bot.set_motor(set_speed, set_speed, set_speed, set_speed)
    while 1:
        left_front = get_GPIO(11)
        right_front = get_GPIO(5)
        left_rear = get_GPIO(19)
        right_rear = get_GPIO(26)
        left_middle = get_GPIO(9)
        right_middle = get_GPIO(6)
        crush_sensor = get_GPIO(13)
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
            if left_front==0 or right_front==0:
                change_speed=20
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
            if left_front==0 or right_front==0:
                change_speed=20
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
            if left_front==0 or right_front==0:
                change_speed=20
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

        if  right_front==0:
            front_condition +=1
        if right_middle==0:
            middle_condition+=1
       
        if front_condition > 0 and middle_condition >0 :
            bot.set_car_motion(0,0,0)
            process_speed_back_road(-20, "time.time()-start_time > 0.7")
            break

def process_speed_longroad_new_40(set_speed):
    start_time = time.time()
    count = 0
    error_a = 0
    last_error_a = 0
    acc_error_a = 0
    front_condition = 0
    middle_condition = 0
    front_condition_right = 0
    p_m = 2
    i_m = 0.06
    p_b = 2
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
    
    useless = return_angle()
    target_angle = return_angle()
    bot.set_motor(set_speed, set_speed, set_speed, set_speed)
    while 1:
        left_front = get_GPIO(11)
        right_front = get_GPIO(5)
        left_rear = get_GPIO(19)
        right_rear = get_GPIO(26)
        left_middle = get_GPIO(9)
        right_middle = get_GPIO(6)
        crush_sensor = get_GPIO(13)
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
            if left_front==0 or right_front==0:
                change_speed=20
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
            if left_front==0 or right_front==0:
                change_speed=20
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
            if left_front==0 or right_front==0:
                change_speed=20
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
        print("left_front",left_front)
        print("right_front",right_front)
       
        if left_front==0 :
            front_condition +=1
        if right_front==0:
            front_condition_right+=1
        if left_middle==0 or right_middle==0:
            middle_condition +=1
        if front_condition > 0 and middle_condition >0 and front_condition_right>0:
            bot.set_car_motion(0,0,0)
            break
def process_speed_longroad_new_50(set_speed):
    start_time = time.time()
    count = 0
    error_a = 0
    last_error_a = 0
    acc_error_a = 0
    front_condition = 0
    middle_condition = 0
    rear_condition=0
    p_m = 2
    i_m = 0.06
    p_b = 2
    i_b = 0.06
    change_speed=40
    
    left_front = get_GPIO(11)
    right_front = get_GPIO(5)
    left_rear = get_GPIO(19)
    right_rear = get_GPIO(26)
    left_middle = get_GPIO(9)
    right_middle = get_GPIO(6)
    pwm_a = pwm_b = pwm_c = pwm_d = 0
    a1 = b1 = c1 = d1 = set_speed
    flag_circle=1
    useless = return_angle()
    target_angle = return_angle()
    bot.set_motor(set_speed, set_speed, set_speed, set_speed)
    flag_right_change=1
    print("5 0")
    while 1:
        left_front = get_GPIO(11)
        right_front = get_GPIO(5)
        left_rear = get_GPIO(19)
        right_rear = get_GPIO(26)
        left_middle = get_GPIO(9)
        right_middle = get_GPIO(6)
        crush_sensor = get_GPIO(13)
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
            if right_front==0:
                flag_right_change=0
            if left_middle==0 and right_middle==1:
                pwm_a+=change_speed
                pwm_b+=change_speed
                pwm_c+=change_speed
                pwm_d+=change_speed
            if right_middle==0 and left_middle==1 and flag_right_change==1:
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
            
            if right_front==0:
                flag_right_change=0
            if left_middle==0 and right_middle==1:
                pwm_a+=change_speed
                pwm_b+=change_speed
                pwm_c+=change_speed
                pwm_d+=change_speed
            if right_middle==0 and left_middle==1 and flag_right_change==1:
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
            pwm_d = p_m * error_d + i_m * acc_error_a_back
            
            if right_front==0:
                flag_right_change=0
            if left_middle==0 and right_middle==1:
                pwm_a+=change_speed
                pwm_b+=change_speed
                pwm_c+=change_speed
                pwm_d+=change_speed
            if right_middle==0 and left_middle==1 and flag_right_change==1:
                pwm_c-=change_speed
                pwm_d-=change_speed
                pwm_a-=change_speed
                pwm_b-=change_speed
           
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)

        if  right_front==0 and flag_circle==1:
            front_condition +=1
            print("right_front")
        if right_middle==0 and flag_circle==1:
            middle_condition +=1
            print("right_middle")
        if flag_circle==0 and left_front==0:
            front_condition+=1
            print("left_front")
        if flag_circle==0 and left_middle==0:
            middle_condition+=1
            print("left_middle")
        
        if front_condition > 0 and middle_condition >0 and flag_circle==1:
            middle_condition=0
            front_condition=0
            flag_circle=0
            a1=b1=c1=d1=10
            print("finish_right")
            start=time.time()
            continue
        if front_condition > 0 and middle_condition >0 and  flag_circle==0 and time.time()-start >0.3:
           print("stop")
           bot.set_car_motion(0,0,0)
           process_speed_back_road(-20,"time.time()-start_time > 0.7")
           break
def process_speed_longroad_new_50_1(set_speed):
    start_time = time.time()
    count = 0
    error_a = 0
    last_error_a = 0
    acc_error_a = 0
    front_condition = 0
    middle_condition = 0
    rear_condition=0
    p_m = 2
    i_m = 0.06
    p_b = 2
    i_b = 0.06
    change_speed=40
    
    left_front = get_GPIO(11)
    right_front = get_GPIO(5)
    left_rear = get_GPIO(19)
    right_rear = get_GPIO(26)
    left_middle = get_GPIO(9)
    right_middle = get_GPIO(6)
    pwm_a = pwm_b = pwm_c = pwm_d = 0
    a1 = b1 = c1 = d1 = set_speed
    flag_circle=1
    useless = return_angle()
    target_angle = return_angle()
    bot.set_motor(set_speed, set_speed, set_speed, set_speed)
    flag_right_change=1
    print("5 0")
    while 1:
        left_front = get_GPIO(11)
        right_front = get_GPIO(5)
        left_rear = get_GPIO(19)
        right_rear = get_GPIO(26)
        left_middle = get_GPIO(9)
        right_middle = get_GPIO(6)
        crush_sensor = get_GPIO(13)
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
            if right_front==0:
                flag_right_change=0
            if left_middle==0 and right_middle==1:
                pwm_a+=change_speed
                pwm_b+=change_speed
                pwm_c+=change_speed
                pwm_d+=change_speed
            if right_middle==0 and left_middle==1 and flag_right_change==1:
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
            
            if right_front==0:
                flag_right_change=0
            if left_middle==0 and right_middle==1:
                pwm_a+=change_speed
                pwm_b+=change_speed
                pwm_c+=change_speed
                pwm_d+=change_speed
            if right_middle==0 and left_middle==1 and flag_right_change==1:
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
            pwm_d = p_m * error_d + i_m * acc_error_a_back
            
            if right_front==0:
                flag_right_change=0
            if left_middle==0 and right_middle==1:
                pwm_a+=change_speed
                pwm_b+=change_speed
                pwm_c+=change_speed
                pwm_d+=change_speed
            if right_middle==0 and left_middle==1 and flag_right_change==1:
                pwm_c-=change_speed
                pwm_d-=change_speed
                pwm_a-=change_speed
                pwm_b-=change_speed
           
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)

        if  right_front==0 and flag_circle==1:
            front_condition +=1
            print("right_front")
        if right_middle==0 and flag_circle==1:
            middle_condition +=1
            print("right_middle")
        if flag_circle==0 and left_front==0:
            front_condition+=1
            print("left_front")
        if flag_circle==0 and left_middle==0:
            middle_condition+=1
            print("left_middle")
        
        if front_condition > 0 and middle_condition >0 and flag_circle==1:
            middle_condition=0
            front_condition=0
            flag_circle=0
            a1=b1=c1=d1=10
            print("finish_right")
            break
        
def process_speed_longroad_new_410(set_speed):
    start_time = time.time()
    count = 0
    error_a = 0
    last_error_a = 0
    acc_error_a = 0
    front_condition_right = 0
    front_condition_left=0
    middle_condition_left = 0
    middle_condition_right = 0
    p_m = 2
    i_m = 0.06
    p_b = 2
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
    
    useless = return_angle()
    target_angle = return_angle()
    bot.set_motor(set_speed, set_speed, set_speed, set_speed)
    while 1:
        left_front = get_GPIO(11)
        right_front = get_GPIO(5)
        left_rear = get_GPIO(19)
        right_rear = get_GPIO(26)
        left_middle = get_GPIO(9)
        right_middle = get_GPIO(6)
        crush_sensor = get_GPIO(13)
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
            if left_front==0 or right_front==0:
                change_speed=20
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
            if left_front==0 or right_front==0:
                change_speed=20
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
            if left_front==0 or right_front==0:
                change_speed=20
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

        if left_front==0 :
            front_condition_left +=1
        if right_front==0:
            front_condition_right+=1
        if left_middle==0:
            middle_condition_left +=1
        if right_middle==0:
            middle_condition_right+=1
        if front_condition_left > 0 and middle_condition_right >0 and front_condition_right>0:
            bot.set_car_motion(0,0,0)
            break
        if front_condition_right > 0 and middle_condition_left >0 and front_condition_left>0:
            bot.set_car_motion(0,0,0)
            break
        #if eval(exit_condition):   # 退出条件
            #bot.set_motor(0, 0, 0, 0)
            #break
def process_speed_high(set_speed,exit_condition):
    start_time = time.time()
    count = 0
    error_a = 0
    last_error_a = 0
    acc_error_a = 0
    p_m = 2.1
    i_m = 0.15
    p_b = 2.1
    i_b = 0.15
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
    useless = return_angle()
    target_angle = return_angle()
    while 1:
        left_front = get_GPIO(11)
        right_front = get_GPIO(5)
        left_rear = get_GPIO(19)
        right_rear = get_GPIO(26)
        left_middle = get_GPIO(9)
        right_middle = get_GPIO(6)
        crush_sensor = get_GPIO(13)
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
            if left_front==0 or right_front==0:
                change_speed=20
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
            if left_rear==0 or right_rear==0:
                change_speed=50
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)
        if error_a > 180:
            error_c = target_angle - now_angle - 360
            acc_error_a = acc_error_a + error_c
            pwm_a = p_b * error_c + i_b * acc_error_a
            pwm_b = p_b * error_c + i_b * acc_error_a
            pwm_c = p_m * error_c + i_m * acc_error_a
            pwm_d = p_m * error_c + i_m * acc_error_a
            if left_front==0 or right_front==0:
                change_speed=20
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
            if left_rear==0 or right_rear==0:
                change_speed=50
            
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)
        if error_a < -180:
            error_d = target_angle - now_angle + 360
            acc_error_a = acc_error_a + error_d
            pwm_a = p_b * error_d + i_b * acc_error_a
            pwm_b = p_b * error_d + i_b * acc_error_a
            pwm_c = p_m * error_d + i_m * acc_error_a
            pwm_d = p_m * error_d + i_m * acc_error_a
            if left_front==0 or right_front==0:
                change_speed=20
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
            if left_rear==0 or right_rear==0:
                change_speed=50
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)

        if pitch < -5:
            count = 1

        if eval(exit_condition):   # 退出条件
            bot.set_motor(0, 0, 0, 0)
            break


def process_speed_longroad_new_60(set_speed):
    start_time = time.time()
    count = 0
    error_a = 0
    last_error_a = 0
    acc_error_a = 0
    front_condition = 0
    middle_condition = 0
    p_m = 2
    i_m = 0.06
    p_b = 2
    i_b = 0.06
    change_speed = 50

    left_front = get_GPIO(11)
    right_front = get_GPIO(5)
    left_rear = get_GPIO(19)
    right_rear = get_GPIO(26)
    left_middle = get_GPIO(9)
    right_middle = get_GPIO(6)
    pwm_a = pwm_b = pwm_c = pwm_d = 0
    a1 = b1 = c1 = d1 = set_speed

    useless = return_angle()
    target_angle = return_angle()
    bot.set_motor(set_speed, set_speed, set_speed, set_speed)
    while 1:
        left_front = get_GPIO(11)
        right_front = get_GPIO(5)
        left_rear = get_GPIO(19)
        right_rear = get_GPIO(26)
        left_middle = get_GPIO(9)
        right_middle = get_GPIO(6)
        crush_sensor = get_GPIO(13)
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
            if left_front == 0 or right_front == 0:
                change_speed = 20
            if left_middle == 0 and right_middle == 1:
                pwm_a += change_speed
                pwm_b += change_speed
                pwm_c += change_speed
                pwm_d += change_speed
            if right_middle == 0 and left_middle == 1:
                pwm_c -= change_speed
                pwm_d -= change_speed
                pwm_a -= change_speed
                pwm_b -= change_speed

            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)
        if error_a > 180:
            error_c = target_angle - now_angle - 360
            acc_error_a = acc_error_a + error_c
            pwm_a = p_b * error_c + i_b * acc_error_a
            pwm_b = p_b * error_c + i_b * acc_error_a
            pwm_c = p_m * error_c + i_m * acc_error_a
            pwm_d = p_m * error_c + i_m * acc_error_a
            if left_front == 0 or right_front == 0:
                change_speed = 20
            if left_middle == 0 and right_middle == 1:
                pwm_a += change_speed
                pwm_b += change_speed
                pwm_c += change_speed
                pwm_d += change_speed
            if right_middle == 0 and left_middle == 1:
                pwm_c -= change_speed
                pwm_d -= change_speed
                pwm_a -= change_speed
                pwm_b -= change_speed

            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)
        if error_a < -180:
            error_d = target_angle - now_angle + 360
            acc_error_a = acc_error_a + error_d
            pwm_a = p_b * error_d + i_b * acc_error_a
            pwm_b = p_b * error_d + i_b * acc_error_a
            pwm_c = p_m * error_d + i_m * acc_error_a
            pwm_d = p_m * error_d + i_m * acc_error_a
            if left_front == 0 or right_front == 0:
                change_speed = 20
            if left_middle == 0 and right_middle == 1:
                pwm_a += change_speed
                pwm_b += change_speed
                pwm_c += change_speed
                pwm_d += change_speed
            if right_middle == 0 and left_middle == 1:
                pwm_c -= change_speed
                pwm_d -= change_speed
                pwm_a -= change_speed
                pwm_b -= change_speed

            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)

        if left_front == 0 or right_front == 0:
            front_condition += 1
        if left_middle == 0 or right_middle == 0:
            middle_condition += 1
        if front_condition > 0 and middle_condition > 0:
            bot.set_car_motion(0, 0, 0)
            break
if __name__ == '__main__':
    useless = return_angle()
    target_angle = return_angle()
    #go_t_mountain(target_angle)
    #process_speed_longroad_new_50(60)
    process_speed_high(40,"time.time() - start_time > 4")
    #process_speed3(90,"time.time() - start_time > 0.1")
    #process_speed(90)
    #bot.set_car_motion(0,0,0)
    #process_speed1(-30,"time.time() - start_time > 0.8")
    #process_speed(20, "-5 < pitch < 5")







