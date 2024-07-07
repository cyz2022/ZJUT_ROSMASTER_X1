from simple_input import *
from imu_information import *
from adjust_speed import *
from go_straight import *
from Rosmaster_Lib import Rosmaster
# from fixed_distance import *
import math
import threading

bot = Rosmaster()

dim = 0.104
l = math.pi*dim


def cross_bridge():
    flag = 0
    flag_rush = 0
    flag_change = 0

    process_speed(80, "-5 < pitch < 5")    # 上桥

    while 1:
        pitch = return_pitch() 
        left_front = get_GPIO(11)
        right_front = get_GPIO(5)
        print("lr",left_front,right_front)     
        if flag_rush == 0:
            start = time.time()
            flag_rush = 1
        else:
            if time.time() - start > 0.1:
                print("speed 20")
                adjust_speed(20, left_front, right_front)
            else:
                print("speed 50")
                adjust_speed(50, left_front, right_front)

        if pitch < -5:
            print("down")
            break

   
    process_speed(5,"-5 < pitch < 5") 

   # while 1:
   #     pitch = return_pitch()
   #     bot.set_motor(0,0,0,0)
   #     if -5 < pitch < 0:
   #         bot.set_car_motion(0,0,0)
   #         print("third")
   #         break


if __name__ == '__main__':
    cross_bridge()
