#ËÙ¶È»·
from Rosmaster_Lib import Rosmaster
import time
import math
import numpy as np
from imu_information import *
bot = Rosmaster()

v_a = 0
v_b = 0
v_c = 0
v_d = 0

def turn_cross(angle,reverse):
    global v_a
    global v_b
    global v_c
    global v_d
    k = 0.0000001636*angle*angle*angle-0.00006563*angle*angle + 0.009362*angle+0.3579
    bot.clear_auto_report_data()
    bot.reset_flash_value()
    bot.create_receive_threading()
    enable = True
    bot.set_auto_report_state(enable, forever=False)
    flag = 0 
    d = 0.104
    l = math.pi*d
    a = 0
    b = 0
    c = 0
    d = 0
    a1 = 80
    b1 = 80
    c1 = 80
    d1 = 80

    old_time = 0
    start_time = time.time()
    error_a = 0
    error_b = 0
    error_c = 0
    error_d = 0
    acc_error_a = 0
    acc_error_b = 0
    acc_error_c = 0
    acc_error_d = 0

    if reverse == -1:
        p_m = 1.3
        p_b = 0.9
    else:
        p_m = 0.9
        p_b = 1.3
    i_m = 0.000006
    i_b = 0.000006		
    pwm_a = 0
    pwm_b = 0
    pwm_c = 0	
    pwm_d = 0
    set_speed = 80	
    if reverse == -1:
        bot.set_motor(a1,b1,-c1,-d1)
    else:
        bot.set_motor(-a1,-b1,c1,d1)
    case = 0
    useless = return_angle()
    now_angle = return_angle()
    if reverse == 1:        
        if now_angle + angle > 180:
            case = 1
            end_angle = angle*k + now_angle - 360
            begin_angle = now_angle
            print("begin_angle",begin_angle)
            print("end_angle",end_angle)
            print("case",case)
        else:
            case = 2
            end_angle = now_angle + angle*k 
            begin_angle = now_angle
            print("begin_angle",begin_angle)
            print("end_angle",end_angle)
            print("case",case)
    else:
        if now_angle - angle < -180:
            case = 3
            begin_angle = now_angle
            end_angle = 180 + (180 + now_angle -angle*k)
            print("begin_angle",begin_angle)
            print("end_angle",end_angle)
            print("case",case)
        else:
            case = 4
            begin_angle = now_angle
            end_angle = now_angle -angle*k
            print("begin_angle",begin_angle)
            print("end_angle",end_angle)
            print("case",case)

    while 1:
        new_a,new_b,new_c,new_d = bot.get_motor_encoder()
        turns = new_a*1.0/1320
        if turns < 1 and flag == 0: 
            flag = 1

        if flag == 1:
            new_time = time.time()
            delta_t = new_time - start_time
            old_time = new_time
            vx,vy,vz = bot.get_motion_data()
            new_a = new_a*1.0/(1320* delta_t)
            new_b = new_b*1.0/(1320* delta_t)
            new_c = new_c*1.0/(1320* delta_t)
            new_d = new_d*1.0/(1320* delta_t)  
            a = new_a
            b = new_b
            c = new_c
            d = new_d
            new_e = (a+d)/2
            new_f = (b+c)/2
            distance = turns * l
            error_a = set_speed - new_e
            error_b = set_speed - new_f
            error_c = set_speed - new_f
            error_d = set_speed - new_e
            acc_error_a = acc_error_a + error_a
            acc_error_b = acc_error_b + error_b
            acc_error_c = acc_error_c + error_c
            acc_error_d = acc_error_d + error_d
            pwm_a = p_b*error_a +i_b*acc_error_a
            pwm_b = p_m*error_b +i_m*acc_error_b
            pwm_c = p_m*error_c +i_m*acc_error_c
            pwm_d = p_b*error_d +i_b*acc_error_d
            if reverse == -1:
                bot.set_motor(pwm_a,pwm_b,-pwm_c,-pwm_d)
            else:
                bot.set_motor(-pwm_a,-pwm_b,pwm_c,pwm_d)

            now_angle = return_angle()
            print("now_angle",now_angle)
      
            if case == 1 and now_angle > end_angle and now_angle < 0:
                bot.set_car_motion(0,0,0)
                break
            elif case == 1 and end_angle > -2 and now_angle < 2 and now_angle < 0:
                bot.set_car_motion(0,0,0)
                break    
                
            elif case == 2 and now_angle > end_angle:
                bot.set_car_motion(0,0,0)
                break
            elif case == 2 and end_angle > 178 and now_angle < -178:
                bot.set_car_motion(0,0,0)
                break

            elif case == 3 and now_angle < end_angle and now_angle > 0:
                bot.set_car_motion(0,0,0)
                break
            elif case == 3 and end_angle < 2 and now_angle > -2 and now_angle < 0:
                bot.set_car_motion(0,0,0)
                break
                
            elif case == 4 and now_angle < end_angle:
                bot.set_car_motion(0,0,0)
                break
            elif case == 4 and end_angle < -178 and now_angle > 178:
                bot.set_car_motion(0,0,0)
                break
    end_now_angle = return_angle()
    print("end_now_angle",end_now_angle)

if __name__ == '__main__':
    # threading.Thread(target = process_speed).start()
    turn_cross(90,-1)
    
