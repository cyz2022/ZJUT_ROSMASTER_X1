#!/usr/bin/env python
# coding:utf-8
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class ROSCtrl:
    def __init__(self):
        self.Joy_active = False
        self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=3)
        self.sub_JoyState = rospy.Subscriber('/JoyState', Bool, self.JoyStateCallback)

    def JoyStateCallback(self, msg):
        if not isinstance(msg, Bool): return
        self.Joy_active = msg.data

    def cancel(self):
        self.pub_vel.publish(Twist())
        self.pub_vel.unregister()
        self.sub_JoyState.unregister()
        rospy.loginfo("Shutting down this node.")


class SinglePID:
    def __init__(self, P=0.1, I=0.0, D=0.1):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        print("init_pid: ", P, I, D)
        self.pid_reset()

    def Set_pid(self, P, I, D):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        print("set_pid: ", P, I, D)
        self.pid_reset()

    def pid_compute(self, target, current):
        self.error = target - current
        self.intergral += self.error
        self.derivative = self.error - self.prevError
        result = self.Kp * self.error + self.Ki * self.intergral + self.Kd * self.derivative
        self.prevError = self.error
        return result

    def pid_reset(self):
        self.error = 0
        self.intergral = 0
        self.derivative = 0
        self.prevError = 0
