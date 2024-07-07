from imu_usb import *
from Rosmaster_Lib import Rosmaster
from RGB import *
import math
import time
bot = Rosmaster()
imu = imuSub()
pitch_corr = 0
v_a = 0
v_b = 0
v_c = 0
v_d = 0


def open_info():
    bot.create_receive_threading()
    enable = True
    bot.set_auto_report_state(enable, forever=False)

def correct_angle(target_angle):
    print("correct")
    now_angle = return_angle()
    delta_angle = (target_angle - now_angle + 360) % 360
    if 5 < delta_angle < 180:
        turn_cross(delta_angle, 1)
    elif 180 < delta_angle < 355:
        turn_cross(360 - delta_angle, -1)

def correct_micro_angle(target_angle):
    print("correct")
    now_angle = return_angle()
    delta_angle = (target_angle - now_angle + 360) % 360
    if 3 < delta_angle < 180:
        turn_cross(delta_angle, 1)
    elif 180 < delta_angle < 357:
        turn_cross(360 - delta_angle, -1)

def turn_cross_platform(angle,reverse):
    global v_a
    global v_b
    global v_c
    global v_d
    bot.clear_auto_report_data()
    bot.reset_flash_value()     
    bot.create_receive_threading()
    enable = True
    bot.set_auto_report_state(enable, forever=False)
    k = 0.0000001636*angle*angle*angle-0.00006563*angle*angle + 0.009362*angle+0.3579
    bot.clear_auto_report_data()
    bot.reset_flash_value()
    flag = 0 
    d = 0.104
    l = math.pi*d
    a = 0
    b = 0
    c = 0
    d = 0
    a1 = -100
    b1 = -100
    c1 = 100
    d1 = 100

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
        p_m = 2
        p_b = 2
    else:
        p_m = 0.9
        p_b = 1.3
    i_m = 0.6
    i_b = 0.6	
    pwm_a = 0
    pwm_b = 0
    pwm_c = 0	
    pwm_d = 0
    set_speed = 0
    banjin = 0	
    if reverse == -1:
        bot.set_motor(a1,b1,c1,d1)
    else:
        bot.set_motor(-a1,-b1,c1,d1)
    case = 0
    useless = return_angle()
    now_angle = return_angle()
    if reverse == 1:        
        if now_angle + angle*k > 180:
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
        if now_angle - angle*k < -180:
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
        print(new_a,new_b,new_c,new_d)
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
            if( vz != 0):
                banjin = vx/vz
            error_a = (set_speed - banjin)*10
            error_b = (set_speed - banjin)*10
            error_c = (set_speed - banjin)*10
            error_d = (set_speed - banjin)*10
            acc_error_a = acc_error_a + error_a
            acc_error_b = acc_error_b + error_b
            acc_error_c = acc_error_c + error_c
            acc_error_d = acc_error_d + error_d
            pwm_a = p_b*error_a +i_b*acc_error_a
            pwm_b = p_m*error_b +i_m*acc_error_b
            pwm_c = p_m*error_c +i_m*acc_error_c
            pwm_d = p_b*error_d +i_b*acc_error_d
            if reverse == -1:
                bot.set_motor(a1+pwm_a,b1+pwm_b,c1+pwm_c,d1+pwm_d)
                print("1",a1+pwm_a,b1+pwm_b,c1+pwm_c,d1+pwm_d)
            else:
                bot.set_motor(-pwm_a,-pwm_b,pwm_c,pwm_d)

            now_angle = return_angle()
            print("now_angle",now_angle)
            threshold = 3
            if -180 + threshold < end_angle < 180 - threshold and end_angle - threshold < now_angle < end_angle + threshold:
                bot.set_car_motion(0,0,0)
                light()
                break
            elif 180 > end_angle > 180 - threshold:
                delta_angle = 180 - end_angle
                if end_angle - threshold < now_angle < 180 or -180 < now_angle < -180 + threshold - delta_angle:
                    bot.set_car_motion(0,0,0)
                    light()
                    break
            if -180 + threshold > end_angle > -180:
                delta_angle = end_angle + 180
                if -180 < now_angle < end_angle + threshold or 180 - delta_angle < now_angle < 180:
                    bot.set_car_motion(0,0,0)
                    light()
                    break
                   
    end_now_angle = return_angle()
   

def turn_cross1(angle,reverse):    
    while(imu.return_angle()[2] == [0.0] *3):
        print(1)
    time.sleep(0.5)
    now_angle = imu.return_angle()[2]
    #print(now_angle)    
    #tolerance = angle*0.02
    tolerance = 1
    compensate = 0.5 #补偿值
    if(reverse == 1):#左转的逻辑如下
        if(now_angle + angle > 180):
            end_angle = angle*0.945 + now_angle - 360
            begin_angle = now_angle
            print("begin_angle",begin_angle)
            print("end_angle",end_angle)
            while(1):
                now_angle = imu.return_angle()[2]
                #print(now_angle)
                if now_angle > 0:
                    k = 1 - (now_angle - begin_angle)/angle + compensate
                else:
                    k = 1 - (180 - begin_angle)/angle - (end_angle - now_angle)/angle + compensate
                #bot.set_motor(int(-70*k),int(-80*k),int(63*k),int(78*k))#改这里！！
                bot.set_motor(int(-100*k),int(-100*k),int(100*k),int(100*k))
                if now_angle > end_angle and now_angle < 0:
                    bot.set_car_motion(0,0,0)
                    break
                elif end_angle > -2 and now_angle < 2 and now_angle < 0:
                    bot.set_car_motion(0,0,0)
                    break                
        else:
            end_angle = now_angle + angle*0.955
            begin_angle = now_angle
            #print("begin_angle",begin_angle)
            #print("end_angle",end_angle)
            while(1):
                
                now_angle = imu.return_angle()[2]
                #print(now_angle)
                k = (end_angle - now_angle)/angle + compensate
                #bot.set_motor(int(-70*k),int(-80*k),int(63*k),int(78*k))#这里也要改！！！和上面同步
                bot.set_motor(int(-100*k),int(-100*k),int(100*k),int(100*k))
                if now_angle > end_angle:
                    bot.set_car_motion(0,0,0)
                    break
                elif end_angle > 178 and now_angle < -178:
                    bot.set_car_motion(0,0,0)
                    break
                    
    else:#右转这次先不调！！！
        if(now_angle - angle < -180):

            end_angle = 180 + (180 + now_angle -angle)
            begin_angle = now_angle

            while(1):
                #print(now_angle)
                now_angle = imu.return_angle()[2]
                if now_angle < 0:
                    k = 1 - (begin_angle - now_angle)/angle + compensate
                else:
                    k = 1 - (180 + begin_angle)/angle - (now_angle - end_angle)/angle + compensate
                bot.set_motor(int(60*k),int(100*k),int(-60*k),int(-100*k))
                if now_angle < end_angle and now_angle > 0:
                    bot.set_car_motion(0,0,0)
                    break
                elif end_angle < 2 and now_angle > -2 and now_angle < 0:
                    bot.set_car_motion(0,0,0)
                    break

        else:
            end_angle = now_angle -angle
            while(1):
                #print(now_angle)
                now_angle = imu.return_angle()[2]
                k = (now_angle - end_angle)/angle + compensate
                bot.set_motor(int(60*k),int(100*k),int(-60*k),int(-100*k))
                if now_angle < end_angle:
                    bot.set_car_motion(0,0,0)
                    break
                elif end_angle < -178 and now_angle > 178:
                    bot.set_car_motion(0,0,0)
                    break

    #while(1):
        #now_angle = imu.return_angle()[2]
        #print(now_angle)
        #if math.fabs(now_angle -end_angle) < tolerance:
        
            #bot.set_car_motion(0,0,0)
      
            #print(math.fabs(now_angle -end_angle))
            #print(imu.return_angle()[2])
            #print(end_angle)
            #break 
def return_pitch():
    global pitch_corr
    while(imu.return_angle() == [0.0] *3):
        print(1)
    pitch_angle = imu.return_angle()[0]
    pitch_angle = pitch_angle*(-1.0) - pitch_corr
    #print(pitch_angle)
    return pitch_angle

def return_roll():
    while(imu.return_angle() == [0.0] *3):
        print(1)
    roll_angle = imu.return_angle()[1]
    #print(pitch_angle)
    return roll_angle

def return_angle():
    while(imu.return_angle() == [0.0] *3):
        print(1)
    angle = imu.return_angle()[2]
    return angle

def return_acc():
    while(imu.return_acc() == [0.0] *3):
        print(1)
    acc = imu.return_acc()[1]
    return acc

def pitch_correction():
    global pitch_corr
    i = 0
    cnt_angle = 0
    useless = return_pitch()
    while i < 10:
        i += 1
        cnt_angle += return_pitch()
    pitch_corr = cnt_angle / 10

if __name__ == "__main__":
    #while(1):
       # print(return_pitch())
    #i = 0
    #pitch_correction()
    #while(i<100):
    #    i+=1
    #    print(return_pitch())
    open_info()
    turn_cross_platform(30,-1)#可以直接跑这个文件测试旋转的效果！！！
    #time.sleep(1)
    #print(return_angle())
  #  start_time = time.time()
  #  number = 0
  #  while 1:
  #      bot.set_motor(30,30,30,30)
  #      a,b,c,d = bot.get_motor_encoder()
  #      print(a,b,c,d)
  #      number += 1
  #      if time.time() - start_time > 1:
  #          break
  #  print("number",number) 
