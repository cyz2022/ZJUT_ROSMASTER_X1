#!/usr/bin/env python
# coding:utf-8
import math
import numpy as np
from common import *
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from dynamic_reconfigure.server import Server
from yahboomcar_laser.cfg import laserTrackerPIDConfig
RAD2DEG = 180 / math.pi

class laserTracker:
    def __init__(self):
        rospy.on_shutdown(self.cancel)
        self.Moving = False
        self.switch = False
        self.ros_ctrl = ROSCtrl()
        self.lin_pid = SinglePID(2.0, 0.0, 2.0)
        self.ang_pid = SinglePID(3.0, 0.0, 5.0)
        self.ResponseDist = rospy.get_param('~targetDist', 1.0)
        Server(laserTrackerPIDConfig, self.dynamic_reconfigure_callback)
        self.laserAngle = 90
        self.priorityAngle = 30  # 40
        self.sub_laser = rospy.Subscriber('/scan', LaserScan, self.registerScan, queue_size=1)

    def cancel(self):
        self.ros_ctrl.pub_vel.publish(Twist())
        self.ros_ctrl.cancel()
        self.sub_laser.unregister()
        rospy.loginfo("Shutting down this node.")

    def registerScan(self, scan_data):
        if not isinstance(scan_data, LaserScan): return
        # 记录激光扫描并发布最近物体的位置（或指向某点）
        # Record the laser scan and publish the position of the nearest object (or point to a point)
        ranges = np.array(scan_data.ranges)
        offset = 0.5
        frontDistList = []
        frontDistIDList = []
        minDistList = []
        minDistIDList = []
        # 按距离排序以检查从较近的点到较远的点是否是真实的东西
        # if we already have a last scan to compare to:
        for i in range(len(ranges)):
            angle = (scan_data.angle_min + scan_data.angle_increment * i) * RAD2DEG
            # if angle > 90: print "i: {},angle: {},dist: {}".format(i, angle, scan_data.ranges[i])
            # 通过清除不需要的扇区的数据来保留有效的数据
            if abs(angle) > (180 - self.priorityAngle):
                if ranges[i] < (self.ResponseDist + offset):
                    frontDistList.append(ranges[i])
                    frontDistIDList.append(angle)
            elif (180 - self.laserAngle) < angle < (180 - self.priorityAngle):
                minDistList.append(ranges[i])
                minDistIDList.append(angle)
            elif (self.priorityAngle - 180) < angle < (self.laserAngle - 180):
                minDistList.append(ranges[i])
                minDistIDList.append(angle)
        # 找到最小距离和最小距离对应的ID
        # Find the minimum distance and the ID corresponding to the minimum distance
        if len(frontDistIDList) != 0:
            minDist = min(frontDistList)
            minDistID = frontDistIDList[frontDistList.index(minDist)]
        else:
            minDist = min(minDistList)
            minDistID = minDistIDList[minDistList.index(minDist)]
        # rospy.loginfo('minDist: {}, minDistID: {}'.format(minDist, minDistID))
        if self.ros_ctrl.Joy_active or self.switch == True:
            if self.Moving == True:
                self.ros_ctrl.pub_vel.publish(Twist())
                self.Moving = not self.Moving
            return
        self.Moving = True
        velocity = Twist()
        if abs(minDist - self.ResponseDist) < 0.1: minDist = self.ResponseDist
        velocity.linear.x = -self.lin_pid.pid_compute(self.ResponseDist, minDist)
        ang_pid_compute = self.ang_pid.pid_compute((180 - abs(minDistID)) / 72, 0)
        if minDistID > 0: velocity.angular.z = -ang_pid_compute
        else: velocity.angular.z = ang_pid_compute
        if ang_pid_compute < 0.02: velocity.angular.z = 0
        self.ros_ctrl.pub_vel.publish(velocity)
        # rospy.loginfo('minDistID: {},ang_pid_compute: {}'.format(minDistID,ang_pid_compute))
        # rospy.loginfo('minDist: {}, linearX: {}'.format(minDist, velocity.linear.x))
        # rospy.loginfo('minDistID: {}, angularSpeed: {}'.format(minDistID, velocity.angular.z))

    def dynamic_reconfigure_callback(self, config, level):
        self.switch = config['switch']
        self.laserAngle = config['laserAngle']
        self.priorityAngle = config['priorityAngle']
        self.ResponseDist = config['ResponseDist']
        self.lin_pid.Set_pid(config['lin_Kp'], config['lin_Ki'], config['lin_Kd'])
        self.ang_pid.Set_pid(config['ang_Kp'], config['ang_Ki'], config['ang_Kd'])
        return config


if __name__ == '__main__':
    rospy.init_node('laser_Tracker', anonymous=False)
    tracker = laserTracker()
    rospy.spin()
