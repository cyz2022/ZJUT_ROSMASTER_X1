#!/usr/bin/env python2
# coding:UTF-8
# Version: V1.0.1
import os
import threading
import RPi.GPIO as GPIO
import time
import serial
from imu_information import *
from imu_usb import *
from Rosmaster_Lib import Rosmaster
from follow_common import *
#from std_msgs.msg import Bool
#from geometry_msgs.msg import Twist
#from dynamic_reconfigure.server import Server
#from dynamic_reconfigure.client import Client
#from sensor_msgs.msg import CompressedImage, LaserScan, Image
#from yahboomcar_linefollw.cfg import LineDetectPIDConfig
# RED: 0, 85, 126, 9, 253, 255
RAD2DEG = 180 / math.pi
bot = Rosmaster()

total_ret = ""
total_frame = ""
camera_flag = 1
pitch_info = 0.0
clock_flag = 1
sensor_flag_old = 1
sensor_res = 2

prev_error_m1 = 0
prev_error_m2 = 0
prev_error_m3 = 0
prev_error_m4 = 0
integral_m1 = 0.0
integral_m2 = 0.0
integral_m3 = 0.0
integral_m4 = 0.0

speed_gear = 2 #巡线速度，2为60 1为35
finish_flag = 1

alpha=0.8
beta=10
#flag = 0
class LineDetect:
    def __init__(self):
        #rospy.on_shutdown(self.cancel)
        #rospy.init_node("LineDetect", anonymous=False)
        #self.imu = imuSub() 
        #self.imu_node = imuNode()
        self.img = None
        self.circle = ()
        self.hsv_range = ((0, 0, 221),
                          (180, 30, 255))
        self.Roi_init = ()
        self.warning = 1
        self.Start_state = True
        #self.dyn_update = False
        self.Buzzer_state = False
        self.select_flags = False
        self.Track_state = 'identify'
        self.windows_name = 'frame'
        #self.ros_ctrl = ROSCtrl()
        self.color = color_follow()
        self.cols, self.rows = 0, 0
        self.Mouse_XY = (0, 0)
        self.img_flip = False
        self.VideoSwitch = True
        self.rear_prev_error = 0.0
        self.rear_integral = 0.0
        self.front_prev_error = 0.0
        self.front_integral = 0.0
        #self.hsv_text = rospkg.RosPack().get_path("yahboomcar_linefollw")+"/scripts/LineFollowHSV.text"
        #Server(LineDetectPIDConfig, self.dynamic_reconfigure_callback)
        
        #self.dyn_client = Client("LineDetect", timeout=60)
        #print(1)
        #self.scale = 500
        #self.FollowLinePID = (45, 0, 30)
        #self.linear = 0.3
        
        #self.PID_init()
        #self.pub_rgb = rospy.Publisher("/linefollw/rgb", Image, queue_size=1)
        #self.pub_Buzzer = rospy.Publisher('/Buzzer', Bool, queue_size=1)
        self.flag1 = -1
        self.cnt = 0
        self.turn_flag = 1
        self.up_flag = 0
        self.break_flag = 0
        self.cancel_flag = 0
  
        #self.total_ret = ""
        #self.total_frame = ""
	#self.imu = imuSub() 
	#self.port = '/dev/ttyUSB0' # USB serial port #/dev/ttyS3
        #self.baud = 9600   # Same baud rate as the INERTIAL navigation module
        #self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=0.5)

        #if self.VideoSwitch == False:
            #print(1)
            #from cv_bridge import CvBridge
            #self.bridge = CvBridge()
            #self.sub_img = rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, self.compressed_callback)
    #def get_frame(self):
        #print("frame1",self.total_frame)
        #return self.total_frame

    def cancel(self):
        #self.Reset()
        self.cancel_flag = 1
        self.VideoSwitch = False
        bot.set_car_motion(0,0,0)
        #time.sleep(5)
        #self.ros_ctrl.cancel()
        #self.sub_img.unregister()
        print(0)
        #self.pub_rgb.unregister()
        #self.pub_Buzzer.unregister()
        
        print ("Shutting down this node.")
        if self.VideoSwitch==False:
            #self.sub_img.unregister()
            print(2)
            
            cv.destroyAllWindows()

    #def turn_cross(self,angle):
        #Gz = 0
        #G_z = 0

	
        #last_angle = self.imu_node.get_yaw() 
        #while(self.imu.return_angle()[2] == [0.0] *3):
            #print(1)
        #last_angle = self.imu.return_angle()[2]
        #print("last_angle",last_angle)
        #tolerance = 2
        #reverse = 1  
        #odom_angle = math.fabs(self.imu_node.get_yaw() - last_angle)
        #odom_angle = math.fabs(self.imu.return_angle()[2] - last_angle) 
        #turn_angle = 0
        #angle *= reverse
       # error = angle - turn_angle  
        #while(math.fabs(error) > tolerance) :
            #while(turn_angle < angle - 4*tolerance):
            
                #bot.set_car_motion(0,0,1)
                #time.sleep(.05)           
                #odom_angle = math.fabs(self.imu.return_angle()[2] - last_angle)          
                #if(odom_angle > 0.5 and odom_angle <100):
                    #turn_angle += odom_angle            
                #print('t',turn_angle)
                #error = angle - turn_angle
                #last_angle = self.imu.return_angle()[2]
            #bot.set_car_motion(0,0,0)
            #rospy.spin()

    #def turn_cross1(self):
        #bot.set_car_motion(0,0,0)
        #Gz = 0
        #while(Gz < 15):
            #bot.create_receive_threading()
            #enable = True
            #bot.set_auto_report_state(enable, forever=False)       
              
            #bot.set_car_motion(0,0,-1)
            #time.sleep(.1)
            #G_X, G_Y ,G_Z = bot.get_gyroscope_data()
            #print('1' , G_Z)
            #print(Gz)
            #Gz += G_Z
            #bot.clear_auto_report_data()
        #bot.set_car_motion(0,0,0)
        #bot.reset_flash_value()

    def t_mountain(self):
        global clock_flag
        global pitch_info
        success_flag = 0
        while pitch_info > -2:
            print(1, pitch_info)
            bot.set_motor(60,100,60,100)
            if pitch_info > 20:
                success_flag = 1
            if pitch_info < 5 and success_flag == 1:
                break
        threading.Thread(target = self.t_mountain_finish).start()

    #用于判断何时回到平地
    def t_mountain_finish(self):
        global finish_flag
        global pitch_info
        while pitch_info < -10: continue
        while abs(pitch_info) > 3: continue
        finish_flag = 1
        
        
    def read_GPIO(self):
        input_pin =11
        sensor_flag = 1
        global sensor_flag_old
        global sensor_res
        GPIO.setmode(GPIO.BCM)  # BCM pin-numbering scheme from Raspberry Pi
        GPIO.setup(input_pin, GPIO.IN)  # set pin as an input pin
        values = []
        try:
            while True:
                value = GPIO.input(input_pin)
                values.append(value)
                if len(values) == 20:
                    if sum(values):
                        sensor_flag = 1 
                    else:
                        sensor_flag = 0
                    values = []
                    if sensor_flag != sensor_flag_old:
                        sensor_res = sensor_flag
                        print("get",sensor_res)
                        sensor_flag_old = sensor_flag  
        finally:
            GPIO.cleanup()
        
        return value

    def process(self, rgb_img, action, cnt_target, cnt_corss):
        global pitch_info
        global alpha
        global beta
        binary = []
        flag = 0
        flag_all = 1
        #rgb_img = cv.resize(rgb_img, (640, 480))
        #rgb_img = cv.flip(rgb_img,0)
        if self.img_flip == True: rgb_img = cv.flip(rgb_img, 1)
        if action == 32: self.Track_state = 'tracking'
        elif action == ord('i') or action == 105: self.Track_state = "identify"
        #elif action == ord('r') or action == 114: self.Reset()
        elif action == ord('q') or action == 113: self.cancel() 
        #if self.Track_state == 'init':
            #cv.namedWindow(self.windows_name, cv.WINDOW_AUTOSIZE)
            #cv.setMouseCallback(self.windows_name, self.onMouse, 0)
            #if self.select_flags == True:
                #if self.Roi_init[0] != self.Roi_init[2] and self.Roi_init[1] != self.Roi_init[3]:
                    #rgb_img, self.hsv_range = self.color.Roi_hsv(rgb_img, self.Roi_init)
                    #self.dyn_update = True
                #else: self.Track_state = 'init'
        #if self.Track_state == "identify":
            #if os.path.exists(self.hsv_text): self.hsv_range = read_HSV(self.hsv_text)
            #else: self.Track_state = 'init'
        if self.Track_state != 'init' and len(self.hsv_range) != 0 and self.turn_flag == 1:
            #self.read_GPIO(24)
            #print(self.ser.read(33))
            #print("imu_data",return_pitch())
            rgb_img, binary, self.circle ,flag= self.color.line_follow(rgb_img, self.hsv_range,self.flag1,cnt_target, cnt_corss,alpha,beta)
            self.flag1 = flag
            #if self.dyn_update == True:
                #print(1)
                #write_HSV(self.hsv_text, self.hsv_range)
                #params = {'Hmin': self.hsv_range[0][0], 'Hmax': self.hsv_range[1][0],
                          #'Smin': self.hsv_range[0][1], 'Smax': self.hsv_range[1][1],
                          #'Vmin': self.hsv_range[0][2], 'Vmax': self.hsv_range[1][2]}
                #self.dyn_client.update_configuration(params)
                #self.dyn_update = False
        if self.Track_state == 'tracking': 
            tt=time.time()
            #bot.set_car_motion(1,0,0)  
            #i=1
            if (pitch_info > 5 and cnt_target == 2 and cnt_corss == 1):
                self.t_mountain()
            elif (pitch_info > 3 and cnt_target == 2 and cnt_corss == 0):
                #self.low_platform()
                i = 1
            else:
                #print(pitch_info)
                if flag == 0 and self.cancel_flag == 0:  
                    flag_all = 0
                elif flag != 0 and self.cancel_flag == 0:
                    threading.Thread(target = self.PID_control, args=(self.front_prev_error, self.front_integral, self.rear_prev_error, self.rear_integral, self.circle[0])).start()
                elif self.cancel_flag == 1:
                    bot.set_car_motion(0,0,0)
         #else:
            #if self.Start_state == True:
                #self.ros_ctrl.pub_cmdVel.publish(Twist())    
                #self.Start_state = False
        return rgb_img, binary, flag_all 

    def low_platform(self):
        global sensor_res
        quit = 0
        cnt_black = 0
        cnt_white = 0
        print("low_platform")
        threading.Thread(target = self.read_GPIO).start()
        while quit == 0:
            if sensor_res == 1 and cnt_black == 0:
                print(0,0)
                bot.set_motor(0,0,0,0)
                sensor_res = 2
                cnt_black += 1
            elif sensor_res == 0 and cnt_white == 0:
                print(1,0)
                sensor_res = 2
                cnt_white += 1
            elif sensor_res == 1 and cnt_black == 1:
                sensor_res = 2
                cnt_black += 1
            elif sensor_res == 0 and cnt_white == 1:
                sensor_res = 2
                cnt_white += 1
                print("zhuan")
                bot.set_car_motion(0,0,0)
                turn_cross(180,1)
                bot.set_motor(59,69,61,71)
                time.sleep(0.5)
                quit = 1

    def PID_control(self, front_prev_error, front_integral, rear_prev_error, rear_integral, Center_x):
    # ???PID??
        #print("SB")
        global speed_gear #巡线速度
        if speed_gear == 2:
            front_Kp = 0.65  # 前轮
            front_Ki = 0  # 前轮
            front_Kd = 0.3  # 前轮
            rear_Kp = 0.65  # 后轮
            rear_Ki = 0  # 后轮
            rear_Kd = 0.3  # 后轮
            k = 0.2  #灵敏度
            v_motor = 60    #基本速度
            v_threshold = 30  #阈值
        elif speed_gear == 1:
            front_Kp = 0.8  # 前轮
            front_Ki = 0  # 前轮
            front_Kd = 0.3  # 前轮
            rear_Kp = 0.8  # 后轮
            rear_Ki = 0  # 后轮
            rear_Kd = 0.3  # 后轮
            k = 1  #灵敏度
            v_motor = 35    #基本速度
            v_threshold = 50  #阈值

        front_error = (320 - Center_x) * k
        
        self.front_integral += front_error
    
        front_derivative = front_error - self.front_prev_error
   
        front_output = front_Kp * front_error + front_Ki * self.front_integral + front_Kd * front_derivative
   
        front_output = max(min(front_output, v_threshold), -1.0*v_threshold)
        rear_error = (320 - Center_x)*k
    
        self.rear_integral += rear_error
   
        rear_derivative = rear_error - self.rear_prev_error
   
        rear_output = rear_Kp * rear_error + rear_Ki * self.rear_integral + rear_Kd * rear_derivative
 
        rear_output = max(min(rear_output, v_threshold), -1.0*v_threshold)
        bot.set_motor(v_motor-front_output-2, v_motor-rear_output-2, v_motor+front_output+4, v_motor+rear_output+3)
        #print(front_error,self.front_integral)
        self.front_prev_error = front_error
        self.rear_prev_error = rear_error


    def speed_loop(self, s1, s2, s3, s4):
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

        m1, m2, m3, m4 = bot.get_motor_encoder()

    # 计算误差
        error_m1 = s1 - m1
        error_m2 = s2 - m2
        error_m3 = s3 - m3
        error_m4 = s4 - m4

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

        bot.set_motor(s1-output_m1, s2-output_m2, s3-output_m3, s4-output_m4)

    # 更新上一次的误差值
        prev_error_m1 = error_m1
        prev_error_m2 = error_m2
        prev_error_m3 = error_m3
        prev_error_m4 = error_m4
    def adapt_light(self,capture):
        global alpha
        global beta
        total_ret,src = capture.read()
        average = np.mean(src)
        print('light',average)
        # print(average)
        light_p=20
        
        while ( average <= 120):
            if(light_p<60):light_p+=20
            capture.set(cv.CAP_PROP_GAIN, light_p)
            total_ret,src = capture.read()
            src = cv.convertScaleAbs(src, None, alpha, beta)
            if(light_p>=60):
                print('1')
                alpha+=0.05
                beta+=5
            average = np.mean(src)
            print('light',average)
      
        return

    def openCapture(self):
        global total_ret
        global total_frame
        global camera_flag
        try:
            capture = cv.VideoCapture(0 , cv.CAP_V4L2)
        
            capture.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*'XVID'))
            capture.set(cv.CAP_PROP_FRAME_WIDTH, 640)
            capture.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
            capture.set(5,120)
            capture.set(cv.CAP_PROP_GAIN, 40)
            #capture.set(10,20)
            capture.set(cv.CAP_PROP_AUTO_EXPOSURE,0)
           #capture.set(cv.CAP_PROP_AUTOWB,0)
            if capture.isOpened():
                self.adapt_light(capture)
            while capture.isOpened():
                 #print(1)
                 total_ret,total_frame = capture.read()
                 #print(total_frame)
                 if camera_flag == 0:
                     break
        except Exception as e:
            print("error",e)
        finally:
            if capture is not None:
                capture.release()
        cv.destroyAllWindows()
    def getting_pitch(self):
        global pitch_info
        pitch_info = return_pitch()

    def change_flag(self):
        global clock_flag
        global finsih_flag
        time.sleep(1)
        while finish_flag == 0: continue
        clock_flag = 1
 
if __name__ == '__main__':
    
    #bot = Rosmaster()
    
    #while(1):
    #imu.return_angle()
    flag_all = 1
    cnt_corss = 0
    cnt_target = 1
    yellow_flag = 0
    line_detect = LineDetect()
    #print(1)
    #line_detect.PID_init()
    #port = '/dev/ttyUSB0' # USB serial port #/dev/ttyS3
    #baud = 9600   # Same baud rate as the INERTIAL navigation module
    #ser1 = serial.Serial('/dev/ttyUSB0', 9600, timeout=0.5)
    print ("HSV: ", line_detect.hsv_range)
    if line_detect.VideoSwitch==False:rospy.spin()
    else:
        #capture = cv.VideoCapture(1 , cv.CAP_V4L2)
        #capture.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*'XVID'))
        #capture.set(cv.CAP_PROP_FRAME_WIDTH, 640)
        #capture.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
        #capture.set(5,120)
        #capture.set(cv.CAP_PROP_GAIN, 20)
        #capture.set(10,20)
        threading.Thread(target = line_detect.openCapture).start()
        threading.Thread(target = line_detect.getting_pitch).start()
        
        time.sleep(5)
        while line_detect.VideoSwitch==True :
            while flag_all !=0:
                                
                #print("imu_data",imu.return_angle()[2])
                start = time.time()
                #ret, frame = capture.read()
                action = cv.waitKey(10) & 0xFF
                frame = total_frame.copy()
                #print("frame",frame)
                #if frame == "":
                #    continue
                frame1, binary, flag_all = line_detect.process(frame, action,cnt_target,cnt_corss)
                #print(binary)
                end = time.time()
                #print("timepro", end - et)
                fps = 1 / (end - start)
                text = "FPS : " + str(int(fps))
                cv.putText(frame1, text, (30, 30), cv.FONT_HERSHEY_SIMPLEX, 0.6, (100, 200, 200), 1)
                cv.imshow('frame',binary)
                cv.imshow('frame2', frame1)
                
                if action == ord('q') or action == 113: 
                    line_detect.break_flag = 1
                    camera_flag = 0
                    break
            if clock_flag:
                cnt_corss += 1
                threading.Thread(target = line_detect.change_flag).start()
            if(line_detect.break_flag == 1):
                break
            if cnt_target == 1 and cnt_corss == 1 and clock_flag:
                finish_flag = 0
                #走长桥函数
                print(1,1)
                clock_flag = 0
                flag_all = 1
                speed_gear = 1
                #bot.set_car_motion(0,0,0)
                #time.sleep(0.5)
            elif cnt_target == 1 and cnt_corss == 2 and clock_flag:
                finish_flag = 0
                clock_flag = 0
                speed_gear = 2
                print(1, 2)
                flag_all = 1
                #上坡的函数
                
                #bot.set_motor(59,69,61,71)
                #time.sleep(1.2)
                #line_detect.low_platform()
                cnt_target += 1
                cnt_corss = 0
            elif cnt_target == 2 and cnt_corss == 1 and clock_flag:
                finish_flag = 0
                clock_flag = 0
                speed_gear = 1
                print(2, 1)
                flag_all = 1
                bot.set_car_motion(0,0,0)
                turn_cross(40, -1)
                
            elif cnt_target == 2 and cnt_corss == 2 and clock_flag:
                speed_gear = 2
                clock_flag = 0
                print(2, 2)
                flag_all = 1
                #bot.set_motor(59,69,61,71)
                #time.sleep(0.2)
                bot.set_car_motion(0,0,0)
                #time.sleep(1)
                turn_cross(30, 1)
            elif cnt_target == 2 and cnt_corss == 3 and clock_flag:
                speed_gear = 2
                clock_flag = 0
                print(2, 3)
                flag_all = 1
                continue
            elif cnt_target == 2 and cnt_corss == 4 and clock_flag:
                speed_gear = 2
                clock_flag = 0
                print(2, 4)
                flag_all = 1
                bot.set_car_motion(0,0,0)
                turn_cross(30,1)
            elif cnt_target == 2 and cnt_corss == 5 and clock_flag:
                speed_gear = 2
                clock_flag = 0
                finish_flag = 0
                print(2, 5)
                flag_all = 1
       		#×ó×ª135
                bot.set_car_motion(0,0,0)
                turn_cross(130,1)
                #上坡的函数
       		#µôÍ·180
            elif cnt_target == 3 and cnt_corss < 4 and clock_flag:
                speed_gear = 2
                clock_flag = 0
                print(3, cnt_corss)
                flag_all = 1
                #bot.set_car_motion(0,0,0)
                #turn_cross(130,-1)
                #time.sleep(1)
                continue
            elif cnt_target == 3 and cnt_corss == 4 and clock_flag:
                speed_gear = 2
                clock_flag = 0
                print(3, cnt_corss)
                flag_all = 1
                cnt_target += 1
                cnt_corss = 0
            elif cnt_target == 4 and cnt_corss == 1 and clock_flag:
                speed_gear = 2
                clock_flag = 0
                flag_all = 1
                turn_cross(45,-1)
            elif cnt_target == 4 and cnt_corss == 2 and clock_flag:
                speed_gear = 2
                clock_flag = 0
                flag_all = 1
                bot.set_car_motion(0, 0, 0)
                #识别颜色
                if color == green:
                    road_select == 2
                    cnt_corss = 8
                else:
                    turn_cross(180,1)
            elif cnt_target == 4 and cnt_corss == 3 and clock_flag:
                speed_gear = 2
                clock_flag = 0
                flag_all = 1
                bot.set_car_motion(0, 0, 0)
                turn_cross(135, 1)
            elif cnt_target == 4 and cnt_corss == 4 and clock_flag:
                speed_gear = 2
                clock_flag = 0
                flag_all = 1
                bot.set_car_motion(0, 0, 0)
                # 识别颜色
                if color == green:
                    road_select == 1
                    cnt_corss = 10
                else:
                    turn_cross(180, 1)
            elif cnt_target == 4 and cnt_corss == 5 and clock_flag:
                speed_gear = 2
                clock_flag = 0
                flag_all = 1
                bot.set_car_motion(0, 0, 0)
                turn_cross(90, -1)
            elif cnt_target == 4 and cnt_corss == 6 and clock_flag:
                speed_gear = 2
                clock_flag = 0
                flag_all = 1
            elif cnt_target == 4 and cnt_corss == 7 and clock_flag:
                speed_gear = 2
                clock_flag = 0
                flag_all = 1
                bot.set_car_motion(0, 0, 0)
                turn_cross(90, -1)
            elif cnt_target == 4 and cnt_corss == 8 and clock_flag:
                speed_gear = 2
                clock_flag = 0
                flag_all = 1
                bot.set_car_motion(0, 0, 0)
                if color == green:
                    road_select == 4
                    cnt_corss = 11
                elif color == yellow:
                    road_select == 5
                    cnt_corss = 11
                else:
                    road_select == 3
                    cnt_corss = 12
                    turn_cross(180, 1)
            elif cnt_target == 4 and cnt_corss == 9 and clock_flag:
                speed_gear = 2
                clock_flag = 0
                flag_all = 1
            elif cnt_target == 4 and cnt_corss == 10 and clock_flag:
                speed_gear = 2
                clock_flag = 0
                flag_all = 1
                bot.set_car_motion(0, 0, 0)
                turn_cross(45, 1)
                cnt_target += 1
                cnt_corss = 0
            elif cnt_target == 4 and cnt_corss == 11 and clock_flag:
                speed_gear = 2
                clock_flag = 0
                flag_all = 1
                bot.set_car_motion(0, 0, 0)
                turn_cross(90, 1)
                cnt_target += 1
                cnt_corss = 0
            elif cnt_target == 4 and cnt_corss == 12 and clock_flag:
                speed_gear = 2
                clock_flag = 0
                flag_all = 1
                bot.set_car_motion(0, 0, 0)
                turn_cross(90, 1)
                cnt_target += 1
                cnt_corss = 0
            elif cnt_target == 4 and cnt_corss == 13 and clock_flag:
                speed_gear = 2
                clock_flag = 0
                flag_all = 1
                bot.set_car_motion(0, 0, 0)
                turn_cross(135, 1)
            elif cnt_target == 4 and cnt_corss == 14 and clock_flag:
                speed_gear = 2
                clock_flag = 0
                flag_all = 1
            elif cnt_target == 4 and cnt_corss == 15 and clock_flag:
                speed_gear = 2
                clock_flag = 0
                flag_all = 1
            elif cnt_target == 4 and cnt_corss == 16 and clock_flag:
                speed_gear = 2
                clock_flag = 0
                flag_all = 1
                bot.set_car_motion(0, 0, 0)
                turn_cross(135, 1)
                cnt_target += 1
                cnt_corss = 0

            elif (road_select == 2 or road_select == 4 or road_select == 5) and cnt_target > 4:
                if cnt_target == 5 and cnt_corss == 1:
                    finish_flag = 0
                    speed_gear = 2
                    clock_flag = 0
                    flag_all = 1
                    #过低平台
                    cnt_target += 1
                    cnt_corss = 0
                elif cnt_target == 6 and cnt_corss == 1:
                    speed_gear = 2
                    clock_flag = 0
                    flag_all = 1
                elif cnt_target == 6 and cnt_corss == 2:
                    finish_flag = 0
                    speed_gear = 1
                    clock_flag = 0
                    flag_all = 1
                    #过两个减速带
                elif cnt_target == 6 and cnt_corss == 3:
                    speed_gear = 2
                    clock_flag = 0
                    flag_all = 1
                elif cnt_target == 6 and cnt_corss == 4:
                    finish_flag = 0
                    speed_gear = 2
                    clock_flag = 0
                    flag_all = 1
                    bot.set_car_motion(0, 0, 0)
                    turn_cross(20,-1)
                    #过跷跷板，上低平台，到六号点位，再过跷跷板
                    #cnt_target += 1
                    #cnt_corss = 0
                elif cnt_target == 7 and cnt_corss == 1:
                    speed_gear = 2
                    clock_flag = 0
                    flag_all = 1
                    bot.set_car_motion(0, 0, 0)
                    turn_cross(90,1)
                elif cnt_target == 7 and cnt_corss == 2:
                    speed_gear = 2
                    clock_flag = 0
                    flag_all = 1
                    #撞直立型景点
                elif cnt_target == 7 and cnt_corss == 3:
                    speed_gear = 2
                    clock_flag = 0
                    flag_all = 1
                    bot.set_car_motion(0, 0, 0)
                    turn_cross(90,1)
                elif cnt_target == 7 and cnt_corss == 4:
                    speed_gear = 2
                    clock_flag = 0
                    flag_all = 1
                    bot.set_car_motion(0, 0, 0)
                    turn_cross(90,1)
                elif cnt_target == 7 and cnt_corss == 5:
                    speed_gear = 2
                    clock_flag = 0
                    flag_all = 1
                    bot.set_car_motion(0, 0, 0)
                    turn_cross(120,-1)
                elif cnt_target == 7 and cnt_corss == 6:
                    finish_flag = 0
                    speed_gear = 2
                    clock_flag = 0
                    flag_all = 1
                    bot.set_car_motion(0, 0, 0)
                    turn_cross(150,-1)
                    #上高平台
                    #cnt_target += 1
                    #cnt_corss = 0
                elif cnt_target == 8 and cnt_corss == 1:
                    finish_flag = 0
                    speed_gear = 1
                    clock_flag = 0
                    flag_all = 1
                    #过梯形山
                elif cnt_target == 8 and cnt_corss == 2:
                    finish_flag = 0
                    speed_gear = 1
                    clock_flag = 0
                    flag_all = 1
                    #过两个减速带
                    #上中平台
                    #cnt_target += 1
                    #cnt_corss = 0
                    #下中平台，过两个减速带
                elif cnt_target == 9 and cnt_corss == 1:
                    speed_gear = 2
                    clock_flag = 0
                    flag_all = 1
                    bot.set_car_motion(0, 0, 0)
                    turn_cross(90,1)
                elif cnt_target == 9 and cnt_corss == 2:
                    speed_gear = 2
                    clock_flag = 0
                    flag_all = 1
                    bot.set_car_motion(0, 0, 0)
                    finish_flag = 0
                    turn_cross(90,1)
                    #直行后过梯形山
                elif cnt_target == 9 and cnt_corss == 3:
                    speed_gear = 2
                    clock_flag = 0
                    flag_all = 1
                    bot.set_car_motion(0, 0, 0)
                    turn_cross(90,1)
                elif cnt_target == 9 and cnt_corss == 4:
                    speed_gear = 1
                    clock_flag = 0
                    flag_all = 1
                    bot.set_car_motion(0, 0, 0)
                    turn_cross(90,1)
                    finish_flag = 0
                    #直行后过梯形山
                elif road_select == 2 and cnt_target == 9 and cnt_corss == 5:
                    speed_gear = 2
                    clock_flag = 0
                    flag_all = 1
                    bot.set_car_motion(0, 0, 0)
                    turn_cross(90,-1)
                elif road_select == 2 and cnt_target == 9 and cnt_corss == 6:
                    speed_gear = 2
                    clock_flag = 0
                    flag_all = 1
                    #撞直立型景点
                elif road_select == 2 and cnt_target == 9 and cnt_corss == 7:
                    speed_gear = 2
                    clock_flag = 0
                    flag_all = 1
                    bot.set_car_motion(0, 0, 0)
                    turn_cross(45,1)
                elif road_select == 2 and cnt_target == 9 and cnt_corss == 8:
                    speed_gear = 2
                    clock_flag = 0
                    flag_all = 1
                elif road_select == 2 and cnt_target == 9 and cnt_corss == 9:
                    speed_gear = 2
                    clock_flag = 0
                    flag_all = 1
                elif road_select == 2 and cnt_target == 9 and cnt_corss == 10:
                    speed_gear = 2
                    clock_flag = 0
                    flag_all = 1
                    bot.set_car_motion(0, 0, 0)
                    turn_cross(30,1)
                    #撞直立型景点，之后调头
                    #turn_cross(150,1)
                elif road_select == 2 and cnt_target == 9 and cnt_corss == 11:
                    speed_gear = 1
                    clock_flag = 0
                    flag_all = 1
                    bot.set_car_motion(0, 0, 0)
                    turn_cross(40,-1)
                    finish_flag = 0
                    #过减速带
                elif road_select == 2 and cnt_target == 9 and cnt_corss == 12:
                    speed_gear = 1
                    clock_flag = 0
                    flag_all = 1
                    bot.set_car_motion(0, 0, 0)
                    turn_cross(40,-1)
                    finish_flag = 0
                    #上低平台
                    print("真的牛逼")

                elif road_select == 4 and cnt_target == 9 and cnt_corss == 5:
                    speed_gear = 2
                    clock_flag = 0
                    flag_all = 1
                    bot.set_car_motion(0, 0, 0)
                    turn_cross(90,-1)
                elif road_select == 4 and cnt_target == 9 and cnt_corss == 6:
                    speed_gear = 2
                    clock_flag = 0
                    flag_all = 1
                    #撞直立型景点
                elif road_select == 4 and cnt_target == 9 and cnt_corss == 7:
                    speed_gear = 2
                    clock_flag = 0
                    flag_all = 1
                elif road_select == 4 and cnt_target == 9 and cnt_corss == 8:
                    speed_gear = 2
                    clock_flag = 0
                    flag_all = 1
                elif road_select == 4 and cnt_target == 9 and cnt_corss == 9:
                    speed_gear = 2
                    clock_flag = 0
                    flag_all = 1
                    bot.set_car_motion(0, 0, 0)
                    turn_cross(90,1)
                elif road_select == 4 and cnt_target == 9 and cnt_corss == 10:
                    speed_gear = 1
                    clock_flag = 0
                    flag_all = 1
                    bot.set_car_motion(0, 0, 0)
                    turn_cross(150,-1)
                    finish_flag = 0
                    #过减速带
                elif road_select == 4 and cnt_target == 9 and cnt_corss == 11:
                    speed_gear = 1
                    clock_flag = 0
                    flag_all = 1
                    bot.set_car_motion(0, 0, 0)
                    turn_cross(40,-1)
                    finish_flag = 0
                    #上低平台
                    print("真的牛逼")
                
                elif road_select == 5 and cnt_target == 9 and cnt_corss == 5:
                    speed_gear = 2
                    clock_flag = 0
                    flag_all = 1
                elif road_select == 5 and cnt_target == 9 and cnt_corss == 6:
                    speed_gear = 2
                    clock_flag = 0
                    flag_all = 1
                    bot.set_car_motion(0, 0, 0)
                    turn_cross(90,-1)
                    #撞直立型景点
                elif road_select == 5 and cnt_target == 9 and cnt_corss == 7:
                    speed_gear = 2
                    clock_flag = 0
                    flag_all = 1
                    #撞直立型景点
                elif road_select == 5 and cnt_target == 9 and cnt_corss == 8:
                    speed_gear = 2
                    clock_flag = 0
                    flag_all = 1
                    bot.set_car_motion(0, 0, 0)
                    turn_cross(45,-1)
                elif road_select == 5 and cnt_target == 9 and cnt_corss == 9:
                    speed_gear = 2
                    clock_flag = 0
                    flag_all = 1
                elif road_select == 5 and cnt_target == 9 and cnt_corss == 10:
                    speed_gear = 2
                    clock_flag = 0
                    flag_all = 1
                elif road_select == 5 and cnt_target == 9 and cnt_corss == 11:
                    speed_gear = 2
                    clock_flag = 0
                    flag_all = 1
                    bot.set_car_motion(0, 0, 0)
                    turn_cross(135,1)
                elif road_select == 5 and cnt_target == 9 and cnt_corss == 12:
                    speed_gear = 1
                    clock_flag = 0
                    flag_all = 1
                    bot.set_car_motion(0, 0, 0)
                    turn_cross(150,-1)
                    finish_flag = 0
                    #过减速带
                elif road_select == 5 and cnt_target == 9 and cnt_corss == 13:
                    speed_gear = 1
                    clock_flag = 0
                    flag_all = 1
                    bot.set_car_motion(0, 0, 0)
                    turn_cross(40,-1)
                    finish_flag = 0
                    #上低平台
                    print("真的牛逼")

            elif (road_select == 1 or road_select ==3) and cnt_target > 4:
                if cnt_target == 5 and cnt_corss == 1:
                    finish_flag = 0
                    speed_gear = 1
                    clock_flag = 0
                    flag_all = 1
                    #过两个减速带
                if cnt_target == 5 and cnt_corss == 2:
                    finish_flag = 0
                    speed_gear = 2
                    clock_flag = 0
                    flag_all = 1
                    #过低平台，到达五号点位
                    cnt_target += 1
                    cnt_corss = 0
                elif cnt_target == 6 and cnt_corss == 1:
                    speed_gear = 2
                    clock_flag = 0
                    flag_all = 1
                elif cnt_target == 6 and cnt_corss == 2:
                    speed_gear = 2
                    clock_flag = 0
                    flag_all = 1
                    bot.set_car_motion(0, 0, 0)
                    turn_cross(90,1)
                elif cnt_target == 6 and cnt_corss == 3:
                    speed_gear = 2
                    clock_flag = 0
                    flag_all = 1
                    #撞直立型景点
                elif cnt_target == 6 and cnt_corss == 4:
                    finish_flag = 0
                    speed_gear = 1
                    clock_flag = 0
                    flag_all = 1
                    bot.set_car_motion(0, 0, 0)
                    turn_cross(90,1)
                    #直行后过梯形山
                elif cnt_target == 6 and cnt_corss == 5:
                    speed_gear = 2
                    clock_flag = 0
                    flag_all = 1
                    bot.set_car_motion(0, 0, 0)
                    turn_cross(90,-1)
                elif cnt_target == 6 and cnt_corss == 6:
                    finish_flag = 0
                    speed_gear = 1
                    clock_flag = 0
                    flag_all = 1
                    bot.set_car_motion(0, 0, 0)
                    turn_cross(90,-1)
                    #直行后过梯形山
                elif cnt_target == 6 and cnt_corss == 7:
                    speed_gear = 2
                    clock_flag = 0
                    flag_all = 1
                    bot.set_car_motion(0, 0, 0)
                    turn_cross(90,-1)
                elif cnt_target == 6 and cnt_corss == 8:
                    finish_flag = 0
                    speed_gear = 1
                    clock_flag = 0
                    flag_all = 1
                    bot.set_car_motion(0, 0, 0)
                    turn_cross(90,-1)
                    #直行后过两个减速带
                    #上中平台，到达七号点位
                    cnt_target += 1
                    cnt_corss = 0
                    #下中平台，过两个减速带
                elif cnt_target == 7 and cnt_corss == 1:
                    finish_flag = 0
                    speed_gear = 1
                    clock_flag = 0
                    flag_all = 1
                    #直行后过梯形山
                elif cnt_target == 7 and cnt_corss == 2:
                    finish_flag = 0
                    speed_gear = 2
                    clock_flag = 0
                    flag_all = 1
                    #上高平台，到达8号点位
                    cnt_target += 1
                    cnt_corss = 0
                elif cnt_target == 8 and cnt_corss == 1:
                    speed_gear = 2
                    clock_flag = 0
                    flag_all = 1
                    bot.set_car_motion(0, 0, 0)
                    turn_cross(135,1)
                elif cnt_target == 8 and cnt_corss == 2:
                    speed_gear = 2
                    clock_flag = 0
                    flag_all = 1
                    bot.set_car_motion(0, 0, 0)
                    turn_cross(135,1)
                elif cnt_target == 8 and cnt_corss == 3:
                    speed_gear = 2
                    clock_flag = 0
                    flag_all = 1
                    bot.set_car_motion(0, 0, 0)
                    turn_cross(90,-1)
                elif cnt_target == 8 and cnt_corss == 4:
                    speed_gear = 2
                    clock_flag = 0
                    flag_all = 1
                    bot.set_car_motion(0, 0, 0)
                    turn_cross(90,-1)
                elif cnt_target == 8 and cnt_corss == 5:
                    speed_gear = 2
                    clock_flag = 0
                    flag_all = 1
                    #撞直立型景点
                elif cnt_target == 8 and cnt_corss == 6:
                    finish_flag = 0
                    speed_gear = 2
                    clock_flag = 0
                    flag_all = 1
                    bot.set_car_motion(0, 0, 0)
                    turn_cross(20,1)
                    #过跷跷板，上低平台，到六号点位，再过跷跷板
                    #cnt_target += 1
                    #cnt_corss = 0
                elif road_select == 1 and cnt_target == 9 and cnt_corss == 1:
                    speed_gear = 2
                    clock_flag = 0
                    flag_all = 1
                    bot.set_car_motion(0, 0, 0)
                    turn_cross(90,1)
                elif road_select == 1 and cnt_target == 9 and cnt_corss == 2:
                    speed_gear = 2
                    clock_flag = 0
                    flag_all = 1
                elif road_select == 1 and cnt_target == 9 and cnt_corss == 3:
                    speed_gear = 2
                    clock_flag = 0
                    flag_all = 1
                    bot.set_car_motion(0, 0, 0)
                    turn_cross(60,1)
                    #撞直立型景点，之后后退
                    #turn_cross(150,1)
                elif road_select == 1 and cnt_target == 9 and cnt_corss == 4:
                    speed_gear = 1
                    clock_flag = 0
                    flag_all = 1
                    bot.set_car_motion(0, 0, 0)
                    turn_cross(40,1)
                    finish_flag = 0
                    #过减速带
                elif road_select == 1 and cnt_target == 9 and cnt_corss == 5:
                    speed_gear = 1
                    clock_flag = 0
                    flag_all = 1
                    bot.set_car_motion(0, 0, 0)
                    turn_cross(40,-1)
                    finish_flag = 0
                    #上低平台
                    print("真的牛逼")


                elif road_select == 3 and cnt_target == 9 and cnt_corss == 1:
                    speed_gear = 2
                    clock_flag = 0
                    flag_all = 1
                    bot.set_car_motion(0, 0, 0)
                    turn_cross(40,1)
                elif road_select == 3 and cnt_target == 9 and cnt_corss == 2:
                    speed_gear = 2
                    clock_flag = 0
                    flag_all = 1
                elif road_select == 3 and cnt_target == 9 and cnt_corss == 3:
                    speed_gear = 2
                    clock_flag = 0
                    flag_all = 1
                elif road_select == 3 and cnt_target == 9 and cnt_corss == 4:
                    speed_gear = 2
                    clock_flag = 0
                    flag_all = 1
                    bot.set_car_motion(0, 0, 0)
                    turn_cross(130,1)
                elif road_select == 3 and cnt_target == 9 and cnt_corss == 5:
                    speed_gear = 1
                    clock_flag = 0
                    flag_all = 1
                    bot.set_car_motion(0, 0, 0)
                    turn_cross(130,-1)
                    finish_flag = 0
                    #过减速带
                elif road_select == 3 and cnt_target == 9 and cnt_corss == 6:
                    speed_gear = 1
                    clock_flag = 0
                    flag_all = 1
                    bot.set_car_motion(0, 0, 0)
                    turn_cross(40,-1)
                    finish_flag = 0
                    #上低平台
                    print("真的牛逼")



            else: flag_all = 1
        #print(3)

