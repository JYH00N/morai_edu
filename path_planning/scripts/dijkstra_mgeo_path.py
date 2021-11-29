#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import rospy

from math import pi,pow,sqrt
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped,Point32,PoseWithCovarianceStamped
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Float32MultiArray 
from morai_msgs.msg import EgoVehicleStatus

current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_path)

from lib.mgeo.class_defs import *

from mgeo_find import find_mgeo_data

class dijkstra_mgeo_path :
    def __init__(self):
        rospy.init_node('dijkstra_mgeo_path', anonymous=True)

        map_data = "kcity"

        # publisher
        self.global_path_pub = rospy.Publisher('global_path',Path, queue_size=1)

        #subscriber
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.statusCB) ## Vehicl Status Subscriber 
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        
        # class
        mgeo_data = find_mgeo_data(map_data)

        # def
        self.is_goal_pose = False
        self.is_status = False
        self.path_pulish = False

        rate = rospy.Rate(10) # 20hz
        while not rospy.is_shutdown():

            if self.is_goal_pose == True and self.is_status == True and self.path_pulish == True:

                mgeo_data.ego_status(self.status_msg)
                mgeo_data.goal_position(self.goal_msg)

                mgeo_data.find_current_ego_link()
                dijkstra_path = mgeo_data.calc_dijkstra_path()

                self.global_path_pub.publish(dijkstra_path)

            if self.path_pulish == False:
                if self.is_goal_pose == True:
                    self.path_pulish = True
                else:
                    rospy.loginfo(" @@@@@ Waiting for goal pose @@@@@ ")

            rate.sleep()

    def goal_callback(self,msg):
        self.goal_msg = msg
        self.goal_x=self.goal_msg.pose.position.x
        self.goal_y=self.goal_msg.pose.position.y
        self.is_goal_pose = True
        self.pre_goal_msg = self.goal_msg

    def statusCB(self,data): ## Vehicle Status Subscriber 
        self.status_msg=data
        self.is_status=True

if __name__ == '__main__':
    try:
        dijkstra_mgeo_path=dijkstra_mgeo_path()
    except rospy.ROSInterruptException:
        pass