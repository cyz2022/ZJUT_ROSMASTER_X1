
from Rosmaster_Lib import Rosmaster
from RGB import *
from imu_information import *
from go_straight import *
import math
import time
bot = Rosmaster()
bot.set_car_motion(0,0.3)
time.sleep(0.5)
