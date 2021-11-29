#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import os,sys
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus
from std_msgs.msg import Float64

class pidControl:
    def __init__(self):
        
        rospy.init_node('pid_controller')

        self.ctrl_pub = rospy.Publisher('/ctrl_cmd',CtrlCmd,queue_size=1)
        self.ctrl_msg = CtrlCmd()

        rospy.Subscriber('/Ego_topic', EgoVehicleStatus, self.ego_callback)
        rospy.Subscriber('/p', Float64, self.p_callback)
        rospy.Subscriber('/i', Float64, self.i_callback)
        rospy.Subscriber('/d', Float64, self.d_callback)
        rospy.Subscriber('/target_vel', Float64, self.target_vel_callback)

        self.p_gain = 0.0
        self.i_gain = 0.0
        self.d_gain = 0.0
        self.prev_error =0
        self.i_control = 0
        self.controlTime = 0.02

        self.is_ego = False
        self.is_target_vel = False
        self.ctrl_msg.longlCmdType = 1
        
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            try:
                if self.is_ego and self.is_target_vel:
                    output = self.pid(self.target_vel, self.ego_vel)
                    if output > 0.0:
                        self.ctrl_msg.accel = output
                        self.ctrl_msg.brake = 0

                    else:
                        self.ctrl_msg.accel = 0
                        self.ctrl_msg.brake = -output
                    self.ctrl_pub.publish(self.ctrl_msg)


            except KeyboardInterrupt:
                break

    def pid(self, target_vel, current_vel):
        error = target_vel - current_vel
        p_control = self.p_gain * error
        self.i_control += self.i_gain * error * self.controlTime
        d_control = self.d_gain * (error - self.prev_error) / self.controlTime
        output = p_control + self.i_control + d_control

        self.prev_error = error

        return output
        
    def ego_callback(self,msg):
        self.is_ego = True
        self.ego_vel = msg.velocity.x * 3.6

    def p_callback(self,msg):
        self.p_gain = msg.data
        self.i_control = 0

    def i_callback(self,msg):
        self.i_gain = msg.data
        self.i_control = 0

    def d_callback(self,msg):
        self.d_gain = msg.data
        self.i_control = 0

    def target_vel_callback(self,msg):
        self.is_target_vel = True
        self.target_vel = msg.data
        self.i_control = 0

if __name__ == '__main__':
    try:
        pid = pidControl()
    except rospy.ROSInitException:
        pass
        