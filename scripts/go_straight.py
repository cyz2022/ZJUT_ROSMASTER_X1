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


def process_speed(set_speed, exit_condition):
    start_time = time.time()
    down_time = 0
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
        crush_sensor = get_GPIO(13)
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

        if pitch < -5 and count == 0:
            count = 1
            down_time = time.time()

        if eval(exit_condition):   # 退出条件
            bot.set_car_motion(0, 0, 0)
            break

def process_speed1(set_speed, exit_condition):
    start_time = time.time()
    error_a = 0
    last_error_a = 0
    acc_error_a = 0
    p_m = 2
    i_m = 0.15
    p_b = 2
    i_b = 0.15
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
            bot.set_motor(10,  -pwm_b, 10,  pwm_d)
            print("1", a1 + pwm_a, "2",  pwm_b, "3", c1 - pwm_c, "4",  pwm_d)
        if error_a > 180:
            error_c = target_angle - now_angle - 360
            acc_error_a = acc_error_a + error_c
            pwm_a = p_b * error_c + i_b * acc_error_a
            pwm_b = p_b * error_c + i_b * acc_error_a
            pwm_c = p_m * error_c + i_m * acc_error_a
            pwm_d = p_m * error_c + i_m * acc_error_a
            bot.set_motor(10, -pwm_b, 10, pwm_d)
            print("1", a1 + pwm_a, "2", pwm_b, "3", c1 - pwm_c, "4",  pwm_d)
        if error_a < -180:
            error_d = target_angle - now_angle + 360
            acc_error_a = acc_error_a + error_d
            pwm_a = p_b * error_d + i_b * acc_error_a
            pwm_b = p_b * error_d + i_b * acc_error_a
            pwm_c = p_m * error_d + i_m * acc_error_a
            pwm_d = p_m * error_d + i_m * acc_error_a
            bot.set_motor(10,  -pwm_b, 10, pwm_d)
            print("1", a1 + pwm_a, "2", pwm_b, "3", c1 - pwm_c, "4", pwm_d)

        if eval(exit_condition):
            bot.set_car_motion(0, 0, 0)
            break



def process_speed_sensor(set_speed, exit_condition):
    start_time = time.time()
    error_a = 0
    last_error_a = 0
    acc_error_a = 0
    p_m = 2.02
    i_m = 0.06
    p_b = 2.03
    i_b = 0.06
    change_speed=90
    left_front = get_GPIO(11)
    right_front = get_GPIO(5)
    left_rear = get_GPIO(19)
    right_rear = get_GPIO(26)
    left_middle = get_GPIO(9)
    right_middle = get_GPIO(6)
    pwm_a = pwm_b = pwm_c = pwm_d = 0
    a1 = b1 = c1 = d1 = set_speed
    bot.set_motor(set_speed, set_speed, set_speed, set_speed)
    target_angle = return_angle()
    while 1:
        left_middle = get_GPIO(9)
        right_middle = get_GPIO(6)
        pitch = return_pitch()
        now_angle = return_angle()
        error_a = target_angle - now_angle
        # print("error",error_a)
        print("left_middle",left_middle)
        print("right_middle",right_middle)
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

            if right_middle==0:
                if -pwm_a >change_speed:
                    pwm_a+=change_speed
                    pwm_b+=change_speed
                else :
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
            if left_middle==0:
                pwm_c+=change_speed
                pwm_d+=change_speed
            if right_middle==0:
                if -pwm_a >change_speed:
                    pwm_a+=change_speed
                    pwm_b+=change_speed
                else :
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
            if left_middle==0:
                pwm_c+=change_speed
                pwm_d+=change_speed
            if right_middle==0:
                if -pwm_a >change_speed:
                    pwm_a+=change_speed
                    pwm_b+=change_speed
                else :
                    pwm_a-=change_speed
                    pwm_b-=change_speed
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)

        if eval(exit_condition):
            bot.set_car_motion(0, 0, 0)
            break



def process_speed_inverse(set_speed, exit_condition):
    start_time = time.time()
    error_a = 0
    last_error_a = 0
    acc_error_a = 0
    p_m = 2
    i_m = 0.15
    p_b = 2.03
    i_b = 0.15
    change_speed=10
    pwm_a = pwm_b = pwm_c = pwm_d = 0
    a1 = b1 = c1 = d1 = set_speed
    bot.set_motor(set_speed, set_speed, set_speed, set_speed)
    useless = return_angle()
    target_angle = return_angle()
    while 1:
        left_middle = get_GPIO(9)
        right_middle = get_GPIO(6)
        pitch = return_pitch()
        now_angle = return_angle()
        print("left_middle",left_middle)
        print("right_middle",right_middle)
        # print("error",error_a)
        if -180 < error_a < 180:
            error_b = target_angle - now_angle
            acc_error_a = acc_error_a + error_b
            pwm_a = p_b * error_b + i_b * acc_error_a
            error_a = target_angle - now_angle
            pwm_b = p_b * error_b + i_b * acc_error_a
            pwm_c = p_m * error_b + i_m * acc_error_a
            pwm_d = p_m * error_b + i_m * acc_error_a
            if left_middle==0 and right_middle==1:   
                
                if -pwm_b >change_speed:
                    pwm_b+=change_speed
                else :
                    pwm_b-=change_speed
                
            if right_middle==0 and left_middle==1:
                pwm_d+=change_speed
            bot.set_motor(10,  -pwm_b, 10,  pwm_d)
            print("1", a1 + pwm_a, "2",  pwm_b, "3", c1 - pwm_c, "4",  pwm_d)
        if error_a > 180:
            error_c = target_angle - now_angle - 360
            acc_error_a = acc_error_a + error_c
            pwm_a = p_b * error_c + i_b * acc_error_a
            pwm_b = p_b * error_c + i_b * acc_error_a
            pwm_c = p_m * error_c + i_m * acc_error_a
            pwm_d = p_m * error_c + i_m * acc_error_a
            if left_middle==0 and right_middle==1:   
                
                if -pwm_b >change_speed:
                    pwm_b+=change_speed
                else :
                    pwm_b-=change_speed
                
            if right_middle==0 and left_middle==1:
                pwm_d+=change_speed
            bot.set_motor(10, -pwm_b, 10, pwm_d)
            print("1", a1 + pwm_a, "2", pwm_b, "3", c1 - pwm_c, "4",  pwm_d)
        if error_a < -180:
            error_d = target_angle - now_angle + 360
            acc_error_a = acc_error_a + error_d
            pwm_a = p_b * error_d + i_b * acc_error_a
            pwm_b = p_b * error_d + i_b * acc_error_a
            pwm_c = p_m * error_d + i_m * acc_error_a
            pwm_d = p_m * error_d + i_m * acc_error_a
            if left_middle==0 and right_middle==1:   
                
                if -pwm_b >change_speed:
                    pwm_b+=change_speed
                else :
                    pwm_b-=change_speed
                
            if right_middle==0 and left_middle==1:
                pwm_d+=change_speed
            bot.set_motor(10,  -pwm_b, 10, pwm_d)
            print("1", a1 + pwm_a, "2", pwm_b, "3", c1 - pwm_c, "4", pwm_d)

        if eval(exit_condition):
            bot.set_car_motion(0, 0, 0)
            break














if __name__ == '__main__':
    time.sleep(5)
    process_speed(20,"count == 1 and -2 <pitch < 5")
  #  t_mountain(60, "count == 1 and 0 < pitch < 5")
    #time.sleep(5)
    #process_speed(100,"time.time() - start_time > 2")
    #process_speed1(20, "-5 < pitch < 5")
#    process_speed1(-30,"time.time() - start_time > 0.8")
#    process_speed(20, "-5 < pitch < 5")







