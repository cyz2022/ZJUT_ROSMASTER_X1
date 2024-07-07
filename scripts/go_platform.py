from go_straight import *
from simple_input import *
from Rosmaster_Lib import Rosmaster
from robot_behavior import *
from imu_information import *
from adjust_speed import *
from simple_input import *
from roadback import *
from imu_round import turn_cross_platform
import os
from go_straight_longroad import *
import threading

bot = Rosmaster()
clock_insure = 0


def insure():  # 用于色标没正确介入的情况
    global clock_insure
    time.sleep(1.5)
    clock_insure = 1
    print("insure run")

def stand_scene():
    dick = get_GPIO(13)
    while dick == 1:
        bot.set_motor(30,30,30,30)
        dick = get_GPIO(13)
    process_speed_longroad(30, "time.time() - start_time > 0.3")
    process_speed_back(-30)

def stand_scene2():
    dick = get_GPIO(13)
    while dick == 1:
        bot.set_motor(30,30,30,30)
        dick = get_GPIO(13)
    process_speed_longroad(40, "time.time() - start_time > 0.3") # 看看撞正效果行不行
    angle = return_angle()
    process_speed_back_road(-30,"time.time() - start_time > 0.3")
    return angle

def low_platform(platform_number):
    global clock_insure
    print("go platform")
    clock_insure = 0
    flag_change = 0
    # 上坡
    process_speed(80, "-5 < pitch < 5")

    # 坡上   
    print("on the platform")
    #process_speed(50,"time.time() - start_time > 1.0")
    angle = stand_scene2()
    bot.set_car_motion(0, 0, 0)
    print("stop")
    process_speed(-40,"time.time() - start_time > 0.3")
    bot.set_car_motion(0, 0, 0)
    voice_behavior(platform_number)
    print("spin")




    start_angle = return_angle()
    if start_angle < 0:
        target_angle = start_angle + 180
    else:
        target_angle = start_angle - 180
    turn_cross(180, 1)
    bot.set_car_motion(0,0,0)
    time.sleep(0.2)
    correct_angle(target_angle)
    # 下坡前
    process_speed(30, "count == 1 and 0 < pitch < 5")
    
def process_speed_middle_front(set_speed, exit_condition):
    target_angle = return_angle()
    start_time = time.time()
    error_a = 0
    last_error_a = 0
    acc_error_a = 0
    p_m = 2.02
    i_m = 0.15
    p_b = 2.1
    i_b = 0.15
    change_speed=60
    left_front = get_GPIO(11)
    
    right_front = get_GPIO(5)
    left_rear = get_GPIO(19)
    right_rear = get_GPIO(26)
    left_middle = get_GPIO(9)
    right_middle = get_GPIO(6)
    pwm_a = pwm_b = pwm_c = pwm_d = 0
    a1 = b1 = c1 = d1 = set_speed
    bot.set_motor(set_speed, set_speed, set_speed, set_speed)
    #target_angle = return_angle()
    print("1",target_angle)
    while 1:
        left_middle = get_GPIO(9)
        right_middle = get_GPIO(6)
        pitch = return_pitch()
        now_angle = return_angle()
        error_a = target_angle - now_angle
        print("error",error_a)
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
                pwm_a+=change_speed
                
                pwm_c+=change_speed
                pwm_d+=change_speed
                pwm_b+=change_speed
            if right_middle==0:
                if -pwm_a >change_speed:
                    pwm_a+=change_speed
                    pwm_b+=change_speed
                    pwm_c-=change_speed
                    pwm_d-=change_speed
                else :
                    pwm_a-=change_speed
                    pwm_b-=change_speed
                    pwm_c-=change_speed
                    pwm_d-=change_speed
                    
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
                pwm_a+=change_speed
                
                pwm_c+=change_speed
                pwm_d+=change_speed
                pwm_b+=change_speed
            if right_middle==0:
                if -pwm_a >change_speed:
                    pwm_a+=change_speed
                    pwm_b+=change_speed
                    pwm_c-=change_speed
                    pwm_d-=change_speed
                else :
                    pwm_a-=change_speed
                    pwm_b-=change_speed
                    pwm_c-=change_speed
                    pwm_d-=change_speed
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
                pwm_a+=change_speed
                
                pwm_c+=change_speed
                pwm_d+=change_speed
                pwm_b+=change_speed
            if right_middle==0:
                if -pwm_a >change_speed:
                    pwm_a+=change_speed
                    pwm_b+=change_speed
                    pwm_c-=change_speed
                    pwm_d-=change_speed
                else :
                    pwm_a-=change_speed
                    pwm_b-=change_speed
                    pwm_c-=change_speed
                    pwm_d-=change_speed
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)

        if eval(exit_condition):
            bot.set_car_motion(0, 0, 0)
            break

def middle_platform():
    #global clock_insure
    # 上坡
    process_speed_middle_front(20,"time.time() - start_time >1")
    process_speed_high_platform(95, "-5 < pitch < 5")
    # 坡上

    process_speed(30,"time.time() - start_time > 1.5 ")
    
    bot.set_car_motion(0, 0, 0)
    print("stop")
    process_speed(-20,"time.time() - start_time > 0.5")  #1s
 
    bot.set_car_motion(0, 0, 0)
    angle = return_angle()
    print("spin") 
    #turn_cross(180, 1)
    #turn_cross(180, 1)
    #turn_cross_platform(30,-1)
    time.sleep(0.1)
    correct_angle(angle)
    process_speed_middle_front(20,"time.time() - start_time >1")
    process_speed_high_platform(100, "time.time() - start_time >4")

    process_speed(-30, "pitch > 5 and time.time() - start_time > 1")

    process_speed_inverse(50, "0 < pitch < 5")
    process_speed(-30, "pitch > 5 and time.time() - start_time > 1")

    process_speed_inverse(50, "0 < pitch < 5")

    #process_speed_back(-30)


def high_platform_1():
    process_speed_high_platform(95, "-5 < pitch < 5")
    # 方法1 process_speed(30, "pitch > 5")
    process_speed(30, "time.time() - start_time > 0.5")

def high_platform_2():
    process_speed_high_platform(95, "-5 < pitch < 5")
    angle = stand_scene2()

    turn_cross_platform(30,-1)
    correct_angle(angle)
    process_speed(-30, "pitch > 5 and time.time() - start_time > 1")
    process_speed_inverse(50, "0 < pitch < 5")
    process_speed(-30, "time.time() - start_time > 0.5")

 def back_platform():

    # 上坡
    process_speed(80, "-5 < pitch < 3")
    bot.set_car_motion(0, 0, 0)
    turn_cross_platform(30, -1)
    time.sleep(2)
    dick = get_GPIO(13)
    while dick == 0:
        continue

def high_platform_3(angle):
    process_speed(-30, "pitch > 5")
    process_speed_inverse(50, "0 < pitch < 5")
    correct_angle(angle)
    process_speed_back_right(30)
    turn_cross(40,-1)
 
#    time.sleep(0.5)
#    angle = return_angle()
    #turn_cross(180,-1)
#    correct_angle(angle)

def start_platform():
    voice_behavior("p")
    #time.sleep(1)
    # 起始平台盲走
    process_speed(30, "count == 1 and 0 < pitch < 5")
    #while 1:
    #    pitch = return_pitch()
    #    bot.set_motor(0, 0, 5, 5)
    #    if -5 < pitch < 5:
    #        break
def down_high_platform():
    time.sleep(5)
    useless = return_angle()
    angle = return_angle()
    process_speed(-30, "pitch > 5")

    process_speed_inverse(20, "-5 < pitch < 5")

    correct_micro_angle(angle)

    process_speed(-30, "pitch > 5")

    process_speed_inverse(20, "-5 < pitch < 5")

def down_middle_platform():
    time.sleep(5)
    process_speed(-30, "pitch > 5")
    #process_speed(-30,"time.time() - start_time > 0.8")
    process_speed_inverse(30, "0 < pitch < 5")

   # process_speed(30, "pitch < -5")
   # jump_start = time.time()
   # slow_times = 0
   # while 1:
   #     pitch = return_pitch()
        # print(pitch)
   #     left_mid = get_GPIO(9)
   #     right_mid = get_GPIO(6)
        # print(time.time()-jump_start)
   #     if time.time() - jump_start < 0.1 and time.time() - jump_start > 0.05:
   #         slow_times += 1
   #         adjust_speed1(-666, left_mid, right_mid)
   #         print("slowing down")
   #         time.sleep(0.1 / slow_times + 0.01)
   #         jump_start = time.time()
        # bot.set_motor(-5,-5,-2,-3)
   #     adjust_speed1(0, left_mid, right_mid)
        # bot.set_motor(-10,-10,-10,-10)
        # if(time.time()-jump_start<0.7):
        # adjust_speed1(-10,left_mid,right_mid)
        # else:
        # adjust_speed1(5,left_mid,right_mid)
   #     if -5 < pitch < 5:
            # bot.set_motor(0,0,0,0)
   #         bot.set_car_motion(0, 0, 0)
   #         print("down")
   #         break

#   pitch = return_pitch()
# adjust_speed1(5, left_mid, right_mid)
#   bot.set_motor(-10,-10,-10,-10)
#   if -5 < pitch < 5:
#       bot.set_car_motion(0,0,0)
#       time.sleep(2)
def openCapture():
    global total_ret
    global total_frame
    global camera_flag
    try:
        capture = cv.VideoCapture('/dev/camera1', cv.CAP_V4L2)  
            #capture = cv.VideoCapture(1, cv.CAP_V4L2)
        capture.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*'XVID'))
            #capture.set(6, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
        capture.set(cv.CAP_PROP_FRAME_WIDTH, 640)
        capture.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
        capture.set(5,120)
        capture.set(cv.CAP_PROP_AUTO_EXPOSURE, 0.25)
        capture.set(cv.CAP_PROP_GAIN, 10)
            #capture.set(10,20)
        capture.set(cv.CAP_PROP_AUTO_EXPOSURE,0)
           #capture.set(cv.CAP_PROP_AUTOWB,0)
        print("open")
        print(capture.isOpened())
        if capture.isOpened():
            self.adapt_light(capture,0)
                 #print(1)
            #print(2)
        while capture.isOpened():
                 #print(3)
            total_ret,frame = capture.read()
                 #print("1",frame)
            while frame is None or frame == "":
                total_ret,frame = capture.read()
                     #print("2",frame)
            total_frame = frame.copy()
                 #print(total_frame)
            if camera_flag == 0:
                break
    except Exception as e:
        print("error",e)
    finally:
        if capture is not None:
            capture.release()
        cv.destroyAllWindows()




def process_speed_high_platform(set_speed, exit_condition):
    start_time = time.time()
    error_a = 0
    last_error_a = 0
    acc_error_a = 0
    p_m = 2.02
    i_m = 0.07
    p_b = 2.1
    i_b = 0.07
    change_speed=60
    left_front = get_GPIO(11)
    delta=20
    right_front = get_GPIO(5)
    left_rear = get_GPIO(19)
    right_rear = get_GPIO(26)
    left_middle = get_GPIO(9)
    right_middle = get_GPIO(6)
    pwm_a = pwm_b = pwm_c = pwm_d = 0
    a1 = b1 = c1 = d1 = set_speed
    bot.set_motor(set_speed, set_speed, set_speed, set_speed)
    target_angle = return_angle()
    print("1",target_angle)
    while 1:
        left_middle = get_GPIO(9)
        right_middle = get_GPIO(6)
        pitch = return_pitch()
        now_angle = return_angle()
        error_a = target_angle - now_angle
        print("error",error_a)
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
                pwm_a+=change_speed
                #pwm_b+=change_speed
                pwm_c+=change_speed
                pwm_d+=change_speed
                pwm_b+=delta
            if right_middle==0:
                if -pwm_a >change_speed:
                    pwm_a+=change_speed
                    pwm_b+=change_speed
                else :
                    pwm_a-=change_speed
                    pwm_b-=change_speed
                    pwm_c-=change_speed
                    pwm_d-=delta
                    #pwm_d-=change_speed
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
                pwm_a+=change_speed
                #pwm_b+=change_speed
                pwm_c+=change_speed
                pwm_d+=change_speed
                pwm_b+=delta
            if right_middle==0:
                if -pwm_a >change_speed:
                    pwm_a+=change_speed
                    pwm_b+=change_speed
                else :
                    pwm_a-=change_speed
                    pwm_b-=change_speed
                    pwm_c-=change_speed
                    pwm_d-=delta
                    #pwm_d-=change_speed
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
                pwm_a+=change_speed
                #pwm_b+=change_speed
                pwm_c+=change_speed
                pwm_d+=change_speed
                pwm_b+=delta
            if right_middle==0:
                if -pwm_a >change_speed:
                    pwm_a+=change_speed
                    pwm_b+=change_speed
                else :
                    pwm_a-=change_speed
                    pwm_b-=change_speed
                    pwm_c-=change_speed
                    pwm_d-=delta
                    #pwm_d-=change_speed
            bot.set_motor(a1 - pwm_a, b1 - pwm_b, c1 + pwm_c, d1 + pwm_d)
            print("1", a1 - pwm_a, "2", b1 - pwm_b, "3", c1 + pwm_c, "4", d1 + pwm_d)

        if eval(exit_condition):
            bot.set_car_motion(0, 0, 0)
            break


if __name__ == '__main__':
   # bot.set_motor(60,60,60,60)
    #process_speed(60,"pitch > 5")
    #high_platform(2)
    #start_platform()
    # low_platform()
    # down()
    #threading.Thread(target = openCapture).start()
    #middle_platform()
    #down_middle_platform()
    bot.create_receive_threading()
    enable = True
    bot.set_auto_report_state(enable, forever=False)
    process_speed(40, "pitch > 5")
    middle_platform()
    #high_platform_1()
    #down_high_platform()
   # bot.set_motor(-30,-30,-30,-30)
    #process_speed(20,"count == 1 and 0 < pitch < 5")
    #process_speed(20,"0")
