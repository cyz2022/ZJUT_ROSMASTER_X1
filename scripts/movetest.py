from Rosmaster_Lib import Rosmaster
from RGB import *
from imu_information import *
from go_straight import *
import math
import time
bot = Rosmaster()
st  = time.time()
bot.set_car_motion(0,0,0.5)
while time.time()-st<5:
    a =1
bot.set_car_motion(0,0,0)
