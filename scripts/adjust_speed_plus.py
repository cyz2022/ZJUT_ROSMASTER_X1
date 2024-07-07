from Rosmaster_Lib import Rosmaster
from simple_input import *
from imu_information import *
import time
import math
import threading
bot = Rosmaster()
new_motorspeed = []
pwm_a = 0
pwm_b = 0
pwm_c = 0	
pwm_d = 0
set_speed = 0
t = 0
def process_speed():
    global new_motorspeed
    global set_speed
    target_angle = return_angle()
    error_a = 0
    last_error_a = 0
    acc_error_a = 0
    p_m = 1
    i_m = 0.1
    p_b = 3
    i_b = 0.7
    global pwm_a 
    global pwm_b 
    global pwm_c 	
    global pwm_d 
    while 1:     
        if set_speed == 30 or set_speed == 20 or set_speed == 50:   
            new_motorspeed = [set_speed,set_speed,set_speed,set_speed]
            now_angle = imu.return_angle()[2]                      
            error_a = target_angle - now_angle
            if (-180 < error_a < 180):
                error_b = target_angle - now_angle
                acc_error_a = acc_error_a + error_b
                pwm_a = p_b*error_b  + i_b*acc_error_a
                pwm_b = p_b*error_b  + i_b*acc_error_a
                pwm_c = p_m*error_b  + i_m*acc_error_a
                pwm_d = p_m*error_b  + i_m*acc_error_a
                new_motorspeed = [set_speed-pwm_a,set_speed-pwm_b,set_speed+pwm_c,set_speed+pwm_d]
                time.sleep(0.1)
            
            if (error_a > 180):
                error_c = target_angle - now_angle - 360
                acc_error_a = acc_error_a + error_c
                pwm_a = p_b*error_c  + i_b*acc_error_a
                pwm_b = p_b*error_c  + i_b*acc_error_a
                pwm_c = p_m*error_c  + i_m*acc_error_a
                pwm_d = p_m*error_c  + i_m*acc_error_a
                new_motorspeed = [set_speed-pwm_a,set_speed-pwm_b,set_speed+pwm_c,set_speed+pwm_d]
                time.sleep(0.1)
            if (error_a < -180):
                error_d = target_angle - now_angle + 360
                acc_error_a = acc_error_a + error_d
                pwm_a = p_b*error_d  + i_b*acc_error_a
                pwm_b = p_b*error_d  + i_b*acc_error_a
                pwm_c = p_m*error_d  + i_m*acc_error_a
                pwm_d = p_m*error_d  + i_m*acc_error_a
                new_motorspeed = [set_speed-pwm_a,set_speed-pwm_b,set_speed+pwm_c,set_speed+pwm_d]
                time.sleep(0.1)
        elif set_speed == 80:
            new_motorspeed = [73,73,87,87]
        elif set_speed == 90:
        #new_motorspeed = [93,93,87,87]
            new_motorspeed = [90,90,90,90]  #跑直线跑直
        #new_motorspeed = [91,91,91,91]  #中平台
        #new_motorspeed = [88,88,93,93]  #高平台
        elif set_speed == 10:
            new_motorspeed = [10,10,10,10]
        elif set_speed == 5:
            new_motorspeed = [5,5,5,5]
        elif set_speed == -10:
            new_motorspeed = [-7,-3,-12,-8]
        elif set_speed == 0:
            new_motorspeed =[-1,-1,1,1]
        elif set_speed ==-666:#平台下坡专用
            new_motorspeed =[-10,-50,-10,-45]
           

def mymotor(target_speed):
    global new_motorspeed
    run_straight(target_speed)
    bot.set_motor(new_motorspeed[0],new_motorspeed[1],new_motorspeed[2],new_motorspeed[3])

def adjust_speed(target_speed,left_front,right_front):
    global new_motorspeed
    global set_speed
    global t
    
    set_speed = target_speed

    if target_speed == 30:
        tolerance = 80.0 * target_speed / 100
    elif target_speed == 90:
      #  tolerance = 4.0 * target_speed / 100 长桥
       # tolerance = 5.0 * target_speed / 100 跷跷板
        tolerance = 3.0 * target_speed / 100
    elif target_speed == 50:
        tolerance = 35.0 * target_speed / 100
    elif target_speed == 60:
        tolerance = 15.0 * target_speed / 100
    elif target_speed == -10:
        tolerance = 40.0 * target_speed / 100        

    if t == 0:
        thread = threading.Thread(target = process_speed)
        thread.start()
        t = 1

    if target_speed > 0:
        if right_front == 0 and left_front == 0:
            bot.set_motor(new_motorspeed[0],new_motorspeed[1],new_motorspeed[2],new_motorspeed[3])
        elif left_front == 0:
            bot.set_motor(new_motorspeed[0] + tolerance,new_motorspeed[1] + tolerance,new_motorspeed[2] - tolerance,new_motorspeed[3] - tolerance)
            print("right")
        elif right_front == 0:
            bot.set_motor(new_motorspeed[0] - tolerance, new_motorspeed[1] - tolerance, new_motorspeed[2] + tolerance, new_motorspeed[3] + tolerance)
            print("left")
        else:
            bot.set_motor(new_motorspeed[0],new_motorspeed[1],new_motorspeed[2],new_motorspeed[3])
    else:
        if left_front == 0:
            bot.set_motor(new_motorspeed[0] + tolerance,new_motorspeed[1] + tolerance,new_motorspeed[2] ,new_motorspeed[3] )
            print("right")
        elif right_front == 0:
            bot.set_motor(new_motorspeed[0] , new_motorspeed[1] , new_motorspeed[2] + tolerance, new_motorspeed[3] + tolerance)
            print("left")
        else:
            bot.set_motor(new_motorspeed[0],new_motorspeed[1],new_motorspeed[2],new_motorspeed[3])

#寻红线
def adjust_speed1(target_speed,left_mid,right_mid):
    global new_motorspeed
    if target_speed == 30:
       # tolerance = 40.0 * target_speed / 100
        tolerance = 15
    if target_speed == 20:
       # tolerance = 40.0 * target_speed / 100
        tolerance = 15
    elif target_speed == 80:
        tolerance = 10.0 * target_speed / 100
    elif target_speed == 90:
        tolerance = 8.0 * target_speed / 100
    elif target_speed == 50:
        tolerance = 20.0 * target_speed / 100
    elif target_speed == 10:
        tolerance = 90.0 * target_speed / 100
    elif target_speed == 5:
        tolerance = 95.0 * target_speed / 100
    elif target_speed == -10:
        tolerance = 40.0 * target_speed / 100
    elif target_speed ==0:
        tolerance =5
    elif target_speed ==-666:
        tolerance =10
    run_straight(target_speed)
    if left_mid == 0:
        bot.set_motor(new_motorspeed[0] - tolerance, new_motorspeed[1] - tolerance, new_motorspeed[2] + tolerance, new_motorspeed[3] + tolerance)
        #time.sleep(0.1)
        print("left")
    elif right_mid == 0:
        bot.set_motor(new_motorspeed[0] + tolerance,new_motorspeed[1] + tolerance,new_motorspeed[2] - tolerance,new_motorspeed[3] - tolerance)
        #time.sleep(0.1)
        print("right")
    else:
        bot.set_motor(new_motorspeed[0],new_motorspeed[1],new_motorspeed[2],new_motorspeed[3])

#寻白线
def adjust_speed2(target_speed):
    while 1:
        tolerance = 24
        right_front = get_GPIO(5)
        bot.set_motor(target_speed - tolerance,target_speed - tolerance,target_speed + tolerance,target_speed + tolerance)
        if right_front == 1:
            break

def adjust_speed_circle(target_speed,left_mid,right_mid):
    global new_motorspeed
    tolerance = 30
    run_straight(target_speed)
    if left_mid == 0:
        bot.set_motor(new_motorspeed[0] - tolerance, new_motorspeed[1] - tolerance, new_motorspeed[2] + tolerance, new_motorspeed[3] + tolerance)
        time.sleep(0.1)
        print("left")
    elif right_mid == 0:
        bot.set_motor(new_motorspeed[0] + tolerance,new_motorspeed[1] + tolerance,new_motorspeed[2] - tolerance,new_motorspeed[3] - tolerance)
        time.sleep(0.1)
        print("right")
    else:
        bot.set_motor(new_motorspeed[0],new_motorspeed[1],new_motorspeed[2],new_motorspeed[3])         
          
       
