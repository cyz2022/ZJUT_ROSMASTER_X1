#!/usr/bin/env python
# coding:utf-8
import math
import numpy as np
import time
from common import *
from time import sleep
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from dynamic_reconfigure.server import Server
from yahboomcar_laser.cfg import laserAvoidPIDConfig
from Rosmaster_Lib import Rosmaster
from std_msgs.msg import String
RAD2DEG = 180 / math.pi

class laserAvoid:
    def __init__(self):
        rospy.on_shutdown(self.cancel)
        self.r = rospy.Rate(20)
        self.Moving = False
        self.switch = False
        self.Right_warning = 0
        self.Left_warning = 0
        self.front_warning = 0
        self.back_warning = 0
        self.ros_ctrl = ROSCtrl()
        Server(laserAvoidPIDConfig, self.dynamic_reconfigure_callback)
        self.linear = 0.5
        self.angular = 1.0
        self.ResponseDist = 0.55
        self.LaserAngle = 30  # 10~180

        self.sub_laser = rospy.Subscriber('/scan', LaserScan, self.registerScan, queue_size=1)
        self.pub = rospy.Publisher('scan_anwser',String ,queue_size = 100)    
    
        #1 means left/right/front is null,could be passed
        self.left_flag = 1
        self.right_flag = 1
        self.front_flag = 1
        self.back_flag = 1
         
        #distance_detect is the disance between the laser and barriers
        self.distance_detect = 5

    def check_left(self):
        return self.left_flag

    def check_right(self):
        return self.right_flag

    def check_front(self):
        return self.front_flag

    def cancel(self):
        self.ros_ctrl.pub_vel.publish(Twist())
        self.ros_ctrl.cancel()
        self.sub_laser.unregister()
        rospy.loginfo("Shutting down this node.")


    def dynamic_reconfigure_callback(self, config, level):
        self.switch = config['switch']
        self.linear = config['linear']
        self.angular = config['angular'] 
        self.LaserAngle = config['LaserAngle']
        self.ResponseDist = config['ResponseDist']
        return config
 
    def registerScan(self, scan_data):
        if not isinstance(scan_data, LaserScan): return
        # 记录激光扫描并发布最近物体的位置（或指向某点）
        # Record the laser scan and publish the position of the nearest object (or point to a point)
        ranges = np.array(scan_data.ranges)
        self.Right_warning = 0
        self.Left_warning = 0
        self.front_warning = 0
        self.back_warning = 0
        # 按距离排序以检查从较近的点到较远的点是否是真实的东西
        # if we already have a last scan to compare to
        for i in range(len(ranges)):
            angle = (scan_data.angle_min + scan_data.angle_increment * i) * RAD2DEG
            # if angle > 90: print "i: {},angle: {},dist: {}".format(i, angle, scan_data.ranges[i])
            # 通过清除不需要的扇区的数据来保留有效的数据
            if  0 < angle < 20 or 0 > angle > -20:
                if ranges[i] < self.ResponseDist: self.back_warning += 1
            if 110 > angle > 70:
                if ranges[i] < self.ResponseDist: self.Right_warning += 1
            if - 110 < angle < -70:
                if ranges[i] < self.ResponseDist: self.Left_warning += 1
            if abs(angle) > 160:
                if ranges[i] <= self.ResponseDist: self.front_warning += 1
           
        # print (self.Left_warning, self.front_warning, self.Right_warning)
      

        # print("front:",self.front_warning,"left:",self.Left_warning,"right:",self.Right_warning,'back',self.back_warning)
        if self.front_warning > self.distance_detect:
            self.front_flag = 1
        else:
            self.front_flag = 0


        if self.Right_warning > self.distance_detect:
            self.right_flag = 1
        else:
            self.right_flag = 0


        if self.Left_warning > self.distance_detect:
            self.left_flag = 1
        else:
            self.left_flag = 0
       
        if self.back_warning > self.distance_detect:
            self.back_flag = 1
        else:
            self.back_flag = 0
        
        output = str(self.front_flag)+str(self.left_flag)+str(self.right_flag)+str(self.back_flag)
        self.pub.publish(output)          
        self.r.sleep	


if __name__ == '__main__':
    rospy.init_node('laser_Avoidance', anonymous=False)
    tracker = laserAvoid()
    rospy.spin()
