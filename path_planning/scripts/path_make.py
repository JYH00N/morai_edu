#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import rospy
import rospkg
from morai_msgs.msg  import EgoVehicleStatus
from math import pi,cos,sin,pi,sqrt,pow
from nav_msgs.msg import Path
import tf
from geometry_msgs.msg import PoseStamped



class path_maker :

    def __init__(self):
        rospy.init_node('path_maker', anonymous=True)

        arg = rospy.myargv(argv=sys.argv)
        

        rospy.Subscriber("/Ego_topic",EgoVehicleStatus, self.status_callback)
        self.global_path_pub= rospy.Publisher('/global_path',Path, queue_size=1)

        self.is_status=False
        self.prev_x = 0
        self.prev_y = 0

        rospack=rospkg.RosPack()
        pkg_path=rospack.get_path('path_planning')
        full_path=pkg_path +'/' + 'path' + '/'+ 'test_path' +'.txt'
        self.f=open(full_path, 'w')

        self.global_path=Path()
        self.global_path.header.frame_id='/map'

        rate=rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.is_status==True :
                self.path_make()
            rate.sleep()    

        self.f.close()
        

    def path_make(self):

        x=self.status_msg.position.x
        y=self.status_msg.position.y
        z=self.status_msg.position.z
        distance=sqrt(pow(x-self.prev_x,2)+pow(y-self.prev_y,2))
        if distance > 0.3:
            data='{0}\t{1}\t{2}\n'.format(x,y,z)
            self.f.write(data)
            self.prev_x=x
            self.prev_y=y
            print(x,y)

            read_pose=PoseStamped()
            read_pose.pose.position.x=x
            read_pose.pose.position.y=y
            read_pose.pose.position.z=z
            read_pose.pose.orientation.x=0
            read_pose.pose.orientation.y=0
            read_pose.pose.orientation.z=0
            read_pose.pose.orientation.w=1
            self.global_path.poses.append(read_pose)
        self.global_path_pub.publish(self.global_path)

    def status_callback(self,msg): ## Vehicl Status Subscriber 
        self.is_status=True
        self.status_msg=msg
        br = tf.TransformBroadcaster()
        br.sendTransform((self.status_msg.position.x, self.status_msg.position.y, self.status_msg.position.z),
                        tf.transformations.quaternion_from_euler(0, 0, self.status_msg.heading/180*pi),
                        rospy.Time.now(),
                        "gps",
                        "map")
        

if __name__ == '__main__':
    try:
        test_track=path_maker()
    except rospy.ROSInterruptException:
        pass

