from Rosmaster_Lib import Rosmaster
from imu_information import *
from simple_input import *
from imu_information import *
from adjust_speed import *
from Rosmaster_Lib import Rosmaster
#from fixed_distance import *
import time
import math
import numpy as np
import threading
import matplotlib.pyplot as plt

bot = Rosmaster()

left_front = get_GPIO(11)
right_front = get_GPIO(5)
left_rear = get_GPIO(19)
right_rear = get_GPIO(26)
left_middle = get_GPIO(9)
right_middle = get_GPIO(6)



def slow_speed(set_speed, exit_condition):
    start_time = time.time()
    error_a = 0
    last_error_a = 0
    acc_error_a = 0
    p_m = 2
    i_m = 0.06
    p_b = 2
    i_b = 0.06
    change_speed=20
    pwm_a = pwm_b = pwm_c = pwm_d = 0
    a1 = b1=c1=d1=set_speed
    left_front = get_GPIO(11)
    right_front = get_GPIO(5)
    left_rear = get_GPIO(19)
    right_rear = get_GPIO(26)
    left_middle = get_GPIO(9)
    right_middle = get_GPIO(6)
    bot.set_motor(set_speed,set_speed, c1, d1)
    target_angle = return_angle()
    flag_left=1
    flag_right=1
    while 1:
        left_front = get_GPIO(11)
        right_front = get_GPIO(5)
        left_rear = get_GPIO(19)
        right_rear = get_GPIO(26)
        left_middle = get_GPIO(9)
        right_middle = get_GPIO(6)
        print("left_middle",left_middle)
        print("right_middle",right_middle)
        print("left_front",left_front)
        print("right_front",right_front)
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
            #1
            if left_front==0:
                flag_left=0
               
                flag_right=1
            if flag_right==0:
                pwm_b-=20
            if right_front==0:
                flag_right=0
                flag_left=1
            if flag_left==0:
                pwm_d+=20
            if flag_left==0 and right_middle==0:
               pwm_a+=change_speed
               pwm_b+=change_speed
            if flag_right==0 and left_middle==0:
                pwm_d-=change_speed
                pwm_c-=change_speed

            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)
        if error_a > 180:
           
            error_c = target_angle - now_angle - 360
            acc_error_a = acc_error_a + error_c
            pwm_a = p_b * error_c + i_b * acc_error_a
            pwm_b = p_b * error_c + i_b * acc_error_a
            pwm_c = p_m * error_c + i_m * acc_error_a
            pwm_d = p_m * error_c + i_m * acc_error_a
            if left_front==0:
                flag_left=0
               
                flag_right=1
            if flag_right==0:
                pwm_b-=20
            if right_front==0:
                flag_right=0
                flag_left=1
            if flag_left==0:
                pwm_d+=20
            if flag_left==0 and right_middle==0:
               pwm_a+=change_speed
               pwm_b+=change_speed
            if flag_right==0 and left_middle==0:
                pwm_d-=change_speed
                pwm_c-=change_speed
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)
        if error_a < -180:
           
            error_d = target_angle - now_angle + 360
            acc_error_a = acc_error_a + error_d
            pwm_a = p_b * error_d + i_b * acc_error_a
            pwm_b = p_b * error_d + i_b * acc_error_a
            pwm_c = p_m * error_d + i_m * acc_error_a
            pwm_d = p_m * error_d + i_m * acc_error_a
            if left_front==0:
                flag_left=0
               
                flag_right=1
            if flag_right==0:
                pwm_b-=20
            if right_front==0:
                flag_right=0
                flag_left=1
            if flag_left==0:
                pwm_d+=20
            if flag_left==0 and right_middle==0:
               pwm_a+=change_speed
               pwm_b+=change_speed
            if flag_right==0 and left_middle==0:
                pwm_d-=change_speed
                pwm_c-=change_speed
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)

        if eval(exit_condition):
            bot.set_car_motion(0, 0, 0)
            break
if __name__ == '__main__':
    slow_speed(50, "time.time() - start_time > 10")
