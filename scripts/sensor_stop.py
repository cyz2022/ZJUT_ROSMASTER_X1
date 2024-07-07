from simple_input import *
from imu_information import *
from adjust_speed import *
from Rosmaster_Lib import Rosmaster
import math
stop_cnt_rear=0
stop_cnt_front=0
stop_cnt_middle=0
def stop(flag,cnt_target,cnt_corss):
    global stop_cnt_rear
    global stop_cnt_front
    global stop_cnt_middle
    if flag>1 or (cnt_target==1 and cnt_corss==0):
        left_front = get_GPIO(11)
        right_front = get_GPIO(5)
        left_rear = get_GPIO(19)
        right_rear = get_GPIO(26)
        left_middle = get_GPIO(9)   
        right_middle = get_GPIO(6)
        print("left_front=",left_front)
        print("right_front=",right_front)
        print("left_rear=",left_rear)
        print("right_rear=",right_rear)
        print("left_middle=",left_middle)
        print("right_middle=",right_middle)
        print("try_stop")
        if right_rear==0 or left_rear==0:
            stop_cnt_rear+=1
        if right_middle==0 or left_middle==0:
            stop_cnt_middle+=1
        if right_front==0 or left_front==0:
            stop_cnt_front+=1
        print("stop_cnt_front=",stop_cnt_front)
        print("stop_cnt_middle=",stop_cnt_middle)
        print("stop_cnt_rear=",stop_cnt_rear)
        
        if stop_cnt_rear>=1 and stop_cnt_middle>=1:
            print("sensor1")
            stop_cnt_rear=0
            stop_cnt_middle=0
            
            return 1
        if stop_cnt_front>=1 and stop_cnt_rear>=1:
            print("sensor2")
            stop_cnt_rear=0
            stop_cnt_middle=0
            return 1
        
        if cnt_target == 4 and cnt_corss == 1 or cnt_target == 4 and cnt_corss == 4:
            if stop_cnt_front >= 1:
                return 1
        return 0
    elif cnt_target==4 and cnt_corss==0:
        left_front = get_GPIO(11)
        right_front = get_GPIO(5)
        left_rear = get_GPIO(19)
        right_rear = get_GPIO(26)
        left_middle = get_GPIO(9)   
        right_middle = get_GPIO(6)
        print("left_front=",left_front)
        print("right_front=",right_front)
        print("left_rear=",left_rear)
        print("right_rear=",right_rear)
        print("left_middle=",left_middle)
        print("right_middle=",right_middle)
        if right_front==0 or left_front==0:
            stop_cnt_front+=1
        if right_middle==0 or left_middle==0:
            stop_cnt_middle+=1
        if stop_cnt_middle>=1 and stop_cnt_front>=1:
            return 1
        return 0
    else:
        return 0
