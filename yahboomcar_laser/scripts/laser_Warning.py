#!/usr/bin/env python
# coding:utf-8
import math
import numpy as np
from common import *
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from dynamic_reconfigure.server import Server
from yahboomcar_laser.cfg import laserWarningPIDConfig
RAD2DEG = 180 / math.pi

class laserWarning:
    def __init__(self):
        rospy.on_shutdown(self.cancel)
        self.Moving = False
        self.switch = False
        self.Buzzer_state = False
        self.ros_ctrl = ROSCtrl()
        self.ang_pid = SinglePID(3.0, 0.0, 5.0)
        Server(laserWarningPIDConfig, self.dynamic_reconfigure_callback)
        self.laserAngle = 70
        self.ResponseDist = 0.5
        self.pub_Buzzer = rospy.Publisher('/Buzzer', Bool, queue_size=1)
        self.sub_laser = rospy.Subscriber('/scan', LaserScan, self.registerScan, queue_size=1)

    def cancel(self):
        self.ros_ctrl.pub_vel.publish(Twist())
        self.ros_ctrl.cancel()
        self.sub_laser.unregister()
        rospy.loginfo('stopMoving!!!')

    def registerScan(self, scan_data):
        if not isinstance(scan_data, LaserScan): return
        # 记录激光扫描并发布最近物体的位置（或指向某点）
        # Record the laser scan and publish the position of the nearest object (or point to a point)
        ranges = np.array(scan_data.ranges)
        # 创建距离列表，将检测范围内的有效距离放入列表中
        # create distance list, put the effective distance within the detection range into the list
        minDistList = []
        # 创建序列号，将有效距离对应的ID放入列表中
        # Create a serial number and place the ID corresponding to the valid distance in the list
        minDistIDList = []
        # 按距离排序以检查从较近的点到较远的点是否是真实的东西
        # if we already have a last scan to compare to:
        for i in range(len(ranges)):
            angle = (scan_data.angle_min + scan_data.angle_increment * i) * RAD2DEG
            # if angle > 90: print "i: {},angle: {},dist: {}".format(i, angle, scan_data.ranges[i])
            # 通过清除不需要的扇区的数据来保留有效的数据
            if abs(angle) > (180 - self.laserAngle):
                minDistList.append(ranges[i])
                minDistIDList.append(angle)
        if len(minDistList) == 0: return
        # 找到最小距离
        # Find the minimum distance
        minDist = min(minDistList)
        # 找到最小距离对应的ID
        # Find the ID corresponding to the minimum distance
        minDistID = minDistIDList[minDistList.index(minDist)]
        if self.ros_ctrl.Joy_active or self.switch == True:
            if self.Moving == True:
                self.ros_ctrl.pub_vel.publish(Twist())
                self.Moving = not self.Moving
            return
        self.Moving = True
        if minDist <= self.ResponseDist:
            if self.Buzzer_state == False:
                b = Bool()
                b.data = True
                self.pub_Buzzer.publish(b)
                self.Buzzer_state = True
        else:
            if self.Buzzer_state == True:
                self.pub_Buzzer.publish(Bool())
                self.Buzzer_state = False
        velocity = Twist()
        # 使用PID算法使得小车稳步移动到对应位置
        # The PID algorithm is used to make the car move to the corresponding position steadily
        ang_pid_compute = self.ang_pid.pid_compute((180 - abs(minDistID)) / 36, 0)
        if minDistID > 0: velocity.angular.z = -ang_pid_compute
        else: velocity.angular.z = ang_pid_compute
        if ang_pid_compute < 0.02: velocity.angular.z = 0
        self.ros_ctrl.pub_vel.publish(velocity)

    def dynamic_reconfigure_callback(self, config, level):
        self.switch = config['switch']
        self.laserAngle = config['laserAngle']
        self.ResponseDist = config['ResponseDist']
        self.ang_pid.Set_pid(config['ang_Kp'], config['ang_Ki'], config['ang_Kd'])
        return config


if __name__ == '__main__':
    rospy.init_node('laser_Warning', anonymous=False)
    tracker = laserWarning()
    rospy.spin()
