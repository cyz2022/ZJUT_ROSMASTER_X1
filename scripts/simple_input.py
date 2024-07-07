#!/usr/bin/env python

# Copyright (c) 2019-2022, NVIDIA CORPORATION. All rights reserved.
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.

import RPi.GPIO as GPIO
import time
import cv2
#from Rosmaster_Lib import Rosmaster

# Pin Definitions
#input_pin = 11  # BCM pin 18, BOARD pin 12

def get_GPIO(input_pin):
    #GPIO.cleanup()
    prev_value = None
    cnt = 0
    #time1 = 0 
    # Pin Setup:
    GPIO.setmode(GPIO.BCM)  # BCM pin-numbering scheme from Raspberry Pi
    GPIO.setup(input_pin, GPIO.IN)  # set pin as an input pin
    values = []
    #print("Starting demo now! Press CTRL+C to exit")
    try:
        while True:
            #a=time.time()
            #cnt+=1
            value = GPIO.input(input_pin)
            values.append(value)
            if len(values) == 20:
                if sum(values):
                    #print(1)  
                    return 1
                else:
                    #print(0)
                    return 0
                values = []
            #b=time.time()-a
            #time1 +=b
            #if value == GPIO.HIGH:
        
                #bot = Rosmaster()
                #bot.set_pwm_servo(3,0)
                #time.sleep(0.5)
                #bot.set_pwm_servo(3,150)
                #value_str = "HIGH"
            #else:
                #value_str = "LOW"

        #time.sleep(1)
    finally:
        #print(cnt)
        GPIO.cleanup()

if __name__ == '__main__':
    input_pin = 13
    while 1:
        print(get_GPIO(5),get_GPIO(11),get_GPIO(6),get_GPIO(9),get_GPIO(19),get_GPIO(13))

