#速度环
from Rosmaster_Lib import Rosmaster
import time
import math
bot = Rosmaster()

prev_error_m1 = 0
prev_error_m2 = 0
prev_error_m3 = 0
prev_error_m4 = 0
integral_m1 = 0.0
integral_m2 = 0.0
integral_m3 = 0.0
integral_m4 = 0.0

d = 0.104
l = math.pi*d

def speed_loop(s1, s2, s3, s4):
    # 初始化PID参数
    global prev_error_m1
    global prev_error_m2
    global prev_error_m3
    global prev_error_m4
    global integral_m1
    global integral_m2
    global integral_m3
    global integral_m4

    Kp = 0.5  # 比例常数
    Ki = 0.2  # 积分常数
    Kd = 0.1  # 微分常数
    k = 30

    val_x,val_y,val_z = bot.get_motion_data()
    print(bot.get_motion_data())
    #print(m1, m2, m3, m4)

    # 计算误差
    error_m1 = (s1 - val_x)*k
    error_m2 = (s2 - val_x)*k
    error_m3 = (s3 - val_x)*k
    error_m4 = (s4 - val_x)*k

    # 计算积分项
    integral_m1 += error_m1
    integral_m2 += error_m2
    integral_m3 += error_m3
    integral_m4 += error_m4

    # 计算微分项
    derivative_m1 = error_m1 - prev_error_m1
    derivative_m2 = error_m2 - prev_error_m2
    derivative_m3 = error_m3 - prev_error_m3
    derivative_m4 = error_m4 - prev_error_m4

    # 计算PID输出
    output_m1 = Kp * error_m1 + Ki * integral_m1 + Kd * derivative_m1
    output_m2 = Kp * error_m2 + Ki * integral_m2 + Kd * derivative_m2
    output_m3 = Kp * error_m3 + Ki * integral_m3 + Kd * derivative_m3
    output_m4 = Kp * error_m4 + Ki * integral_m4 + Kd * derivative_m4

    bot.set_motor(output_m1, output_m2, output_m3, output_m4)

    # 更新上一次的误差值
    prev_error_m1 = error_m1
    prev_error_m2 = error_m2
    prev_error_m3 = error_m3
    prev_error_m4 = error_m4

def move(speed,method,distance):
    global d
    global l
    turns = 0
    flag = 0
    now_distance = 0
    
    while 1:
        bot.set_motor(speed*method,speed*method,speed*method,speed*method)       
        #bot.set_car_motion(speed*method,0,0)
        a,b,c,d = bot.get_motor_encoder()            
        turns = a*1.0/1320
        if math.fabs(turns) < 1 and flag == 0: 
            flag = 1
        if flag == 1:
            a = a*1.0/1320
            b = b*1.0/1320
            c = c*1.0/1320
            d = d*1.0/1320   
            now_distance = turns * l
            print(now_distance)
            if(now_distance == 0.0):
                bot.set_motor(speed*method,speed*method,speed*method,speed*method)
            if (math.fabs(now_distance) > distance):
                bot.set_car_motion(0,0,0)
                break   

if __name__ == '__main__':
    #bot.clear_auto_report_data()
    bot.reset_flash_value()
    time.sleep(0.5)
    bot.create_receive_threading()
    enable = True
    bot.set_auto_report_state(enable, forever=False)
    move(100,1,1)
    time.sleep(0.5)
    bot.reset_flash_value()
    move(100,-1,1) 
    
  

