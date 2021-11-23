#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
from math import cos,sin,pi,sqrt,pow,atan2
from geometry_msgs.msg import Point,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry,Path
from morai_msgs.msg import CtrlCmd
from morai_msgs.msg  import EgoVehicleStatus
import numpy as np
import tf
from tf.transformations import euler_from_quaternion,quaternion_from_euler


class pure_pursuit :
    def __init__(self):
        rospy.init_node('pure_pursuit', anonymous=True)

        rospy.Subscriber("local_path", Path, self.path_callback)                                    # 차량의 Local Path Subscribe
        rospy.Subscriber("/Ego_topic",EgoVehicleStatus, self.status_callback)                       # Ego 차량 Status Subscribe
    
        self.ctrl_cmd_pub = rospy.Publisher('ctrl_cmd',CtrlCmd, queue_size=1)                       # 차량 제어 Ctrl Cmd Publisher
        self.ctrl_cmd_msg=CtrlCmd()                                                                 # Morai 차량 제어 Ros Msg 정의
        self.ctrl_cmd_msg.longlCmdType=2                                                            # 제어 방식 결정 Index 2 : Velocity control
    
        self.is_path=False                                                                          # Path data 확인
        self.is_status=False                                                                        # Ego Status data 확인
    
        self.forward_point=Point()                                                                  # 
        self.current_postion=Point()                                                                # Ego 차량의 현재 위치 Status 저장
        self.is_look_forward_point=False                                                            # 차량의 Look Forward Point 계산 정보
        self.vehicle_length=1                                                                       # 차량의 크기 정보 Wheelbase
        self.lfd=3                                                                                  # Look Forward Distance

        rate = rospy.Rate(30) # 30hz
        while not rospy.is_shutdown():

            if self.is_path ==True and self.is_status==True :                                       # Path, Ego Status 확인
                
                vehicle_position=self.current_postion                                           
                self.is_look_forward_point= False

                ## Map 기준 좌표계와 차량 기준 좌표계 좌표 변환
                translation=[vehicle_position.x, vehicle_position.y]
                t=np.array([
                        [cos(self.vehicle_yaw), -sin(self.vehicle_yaw),translation[0]],
                        [sin(self.vehicle_yaw),cos(self.vehicle_yaw),translation[1]],
                        [0                    ,0                    ,1            ]])
                det_t=np.array([
                       [t[0][0],t[1][0],-(t[0][0]*translation[0]+t[1][0]*translation[1])],
                       [t[0][1],t[1][1],-(t[0][1]*translation[0]+t[1][1]*translation[1])],
                       [0      ,0      ,1                                               ]])

                for num,i in enumerate(self.path.poses) :                                           # Path data read     
                    path_point=i.pose.position

                    global_path_point=[path_point.x,path_point.y,1]                             
                    local_path_point=det_t.dot(global_path_point)                                   # map 기준 좌표 데이터를 차량 기준 좌표 데이터로 변환
                    if local_path_point[0]>0 :
                        dis=sqrt(pow(local_path_point[0],2)+pow(local_path_point[1],2))
                        if dis>= self.lfd :                                                         # 특정 Point 와 Vehicle 사이 거리가 Lfd 보다 같거나 크다면 그 위치를 Look Forward Point 로 지정
                            self.forward_point=path_point
                            self.is_look_forward_point=True
                            break
                
                theta=atan2(local_path_point[1],local_path_point[0])                                # 차량 현재 위치에서 Look Forward Point 까지 차량 기준 각도 계산
                if self.is_look_forward_point :
                    self.ctrl_cmd_msg.steering=atan2((2*self.vehicle_length*sin(theta)),self.lfd)   # Purpursuit 계산 tan^-1(2 * wheel_base * sin(theta) / Look forward distance)
                    self.ctrl_cmd_msg.velocity=20.0                                                 # Vehicle Target vel = 20 km/h
                    print(self.ctrl_cmd_msg.steering)
                else : 
                    print("no found forward point")
                    self.ctrl_cmd_msg.steering=0.0
                    self.ctrl_cmd_msg.velocity=0.0

                self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
            rate.sleep()

    def path_callback(self,msg):
        self.is_path=True
        self.path=msg  
        if len(self.path.poses) < 5 :
            self.is_path=False

    def status_callback(self,msg): ## Vehicl Status Subscriber 
        self.is_status=True
        self.status_msg=msg

        br = tf.TransformBroadcaster()
        br.sendTransform((self.status_msg.position.x, self.status_msg.position.y, self.status_msg.position.z),
                        tf.transformations.quaternion_from_euler(0, 0, self.status_msg.heading/180*pi),
                        rospy.Time.now(),
                        "gps",
                        "map")
        
        self.vehicle_yaw= self.status_msg.heading/180*pi

        self.current_postion.x=self.status_msg.position.x
        self.current_postion.y=self.status_msg.position.y

if __name__ == '__main__':
    try:
        test_track=pure_pursuit()
    except rospy.ROSInterruptException:
        pass
