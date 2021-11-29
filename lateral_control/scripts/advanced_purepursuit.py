#!/usr/bin/env python
# -*- coding: utf-8 -*-

from numpy.testing._private.utils import temppath
import rospy
import rospkg
from math import cos,sin,pi,sqrt,pow,atan2
from geometry_msgs.msg import Point,PoseStamped
from nav_msgs.msg import Odometry,Path
from morai_msgs.msg import CtrlCmd
from morai_msgs.msg  import EgoVehicleStatus,ObjectStatusList
import numpy as np
import tf
from tf.transformations import euler_from_quaternion,quaternion_from_euler


class pure_pursuit :
    def __init__(self):
        rospy.init_node('pure_pursuit', anonymous=True)

        rospy.Subscriber("/global_path", Path, self.global_path_callback)                                   # 차량의 Local Path Subscribe

        rospy.Subscriber("local_path", Path, self.path_callback)                                    # 차량의 Local Path Subscribe
        rospy.Subscriber("/Ego_topic",EgoVehicleStatus, self.status_callback)                       # Ego 차량 Status Subscribe
        rospy.Subscriber("/Object_topic",ObjectStatusList,self.object_callback)
    
        self.ctrl_cmd_pub = rospy.Publisher('ctrl_cmd',CtrlCmd, queue_size=1)                       # 차량 제어 Ctrl Cmd Publisher
        self.ctrl_cmd_msg=CtrlCmd()                                                                 # Morai 차량 제어 Ros Msg 정의
        self.ctrl_cmd_msg.longlCmdType=1                                                            # 제어 방식 결정 Index 2 : Velocity contro
    
        self.is_path=False                                                                          # Path data 확인
        self.is_status=False                                                                        # Ego Status data 확인
        self.is_global_path=False
        self.is_obj = False
        self.is_lattice_planning = False

        self.forward_point=Point()                                                                  # 
        self.current_postion=Point()                                                                # Ego 차량의 현재 위치 Status 저장
        self.is_look_forward_point=False                                                            # 차량의 Look Forward Point 계산 정보
        self.vehicle_length=2                                                                       # 차량의 크기 정보 Wheelbase
        self.lfd=5                                                                                  # Look Forward Distance
        self.min_lfd=5
        self.max_lfd=30

        CAR_MAX_SPEED = 80

        #class_import
        self.pid = pidControl()
        self.vel_planning = velocityPlanning(CAR_MAX_SPEED/3.6, 0.15)
        self.vo = vaildObject()
        self.cc = cruiseControl(0.5,1.0)
        self.lattice = latticePlanner()

        rate = rospy.Rate(30) # 30hz
        while not rospy.is_shutdown():

            if self.is_path ==True and self.is_status==True and self.is_global_path:                                       # Path, Ego Status 확인
                
                vehicle_position=self.current_postion                                           
                self.is_look_forward_point= False
                
                
                self.lfd = self.status_msg.velocity.x
                if self.lfd < self.min_lfd :
                    self.lfd = self.min_lfd 
                elif self.lfd > self.max_lfd : 
                    self.lfd = self.max_lfd

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

                if self.is_lattice_planning:
                    ref_path = lattice_path[lattice_path_num]
                else:
                    ref_path = self.path

                for num,i in enumerate(ref_path.poses) :                                                 # Path data read     
                    path_point=i.pose.position

                    global_path_point=[path_point.x,path_point.y,1]                                 
                    local_path_point=det_t.dot(global_path_point)                                          # map 기준 좌표 데이터를 차량 기준 좌표 데이터로 변환
                    if local_path_point[0]>0 :  
                        dis=sqrt(pow(local_path_point[0],2)+pow(local_path_point[1],2))
                        if dis>= self.lfd :                                                                   # 특정 Point 와 Vehicle 사이 거리가 Lfd 보다 같거나 크다면 그 위치를 Look Forward Point 로 지정
                            self.forward_point=path_point
                            self.is_look_forward_point=True
                            break
                theta=atan2(local_path_point[1],local_path_point[0])                                        # 차량 현재 위치에서 Look Forward Point 까지 차량 기준 각도 계산
                if self.is_look_forward_point :     
                    self.ctrl_cmd_msg.steering=atan2((2*self.vehicle_length*sin(theta)),self.lfd)           # Purpursuit 계산 tan^-1(2 * wheel_base * sin(theta) / Look forward distance)

                    self.current_waypoint = self.get_current_waypoint(self.status_msg,self.global_path)
                    target_vel = self.vel[self.current_waypoint]

                    result = self.vo.calc_vaild_obj(self.status_msg,self.object_data)
                    global_npc_info = result[0] 
                    local_npc_info = result[1] 
                    global_ped_info = result[2] 
                    local_ped_info = result[3] 
                    global_obs_info = result[4] 
                    local_obs_info = result[5] 

                    self.cc.checkObject(self.path, global_npc_info, local_npc_info
                                                        ,global_ped_info, local_ped_info
                                                        ,global_obs_info, local_obs_info)
                                                        
                    cc_vel = self.cc.acc(local_npc_info, local_ped_info, local_obs_info,
                                            self.status_msg.velocity.x, CAR_MAX_SPEED)

                    lattice_path = self.lattice.latticePlanner(self.path,self.status_msg)
                    self.is_lattice_planning = self.lattice.checkObject(self.path, global_obs_info, lattice_path)
                    if self.is_lattice_planning:
                        lattice_path_num = self.lattice.collision_check(global_obs_info, lattice_path)
                        output = self.pid.pid(min(target_vel/3,cc_vel), self.status_msg.velocity.x)

                    else:
                        output = self.pid.pid(min(target_vel,cc_vel),self.status_msg.velocity.x)

                    if output > 0.0:
                        self.ctrl_cmd_msg.accel = output
                        self.ctrl_cmd_msg.brake = 0.0
                    else:
                        self.ctrl_cmd_msg.accel = 0.0
                        self.ctrl_cmd_msg.brake = -output
                    
                else : 
                    print("no found forward point")
                    self.ctrl_cmd_msg.steering=0.0
                    self.ctrl_cmd_msg.velocity=1.0

                self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
            rate.sleep()

    def object_callback(self,msg):
        self.is_obj=True
        self.object_data = msg

    def path_callback(self,msg):
        self.is_path=True
        self.path=msg  
        if len(self.path.poses) < 5 :
            self.is_path=False

    def global_path_callback(self,msg):
        self.global_path = msg
        self.vel = self.vel_planning.curvedBaseVelocity(self.global_path, 50)
        self.is_global_path = True

    def get_current_waypoint(self,ego_status,global_path):
        min_dist = float('inf')        
        currnet_waypoint = -1
        for i,pose in enumerate(global_path.poses):
            dx = ego_status.position.x - pose.pose.position.x
            dy = ego_status.position.y - pose.pose.position.y

            dist = sqrt(pow(dx,2)+pow(dy,2))
            if min_dist > dist :
                min_dist = dist
                currnet_waypoint = i
        return currnet_waypoint       


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

class velocityPlanning:
    def __init__ (self,car_max_speed, road_friciton):
        self.car_max_speed = car_max_speed
        self.road_friction = road_friciton

    def curvedBaseVelocity(self, gloabl_path, point_num):
        out_vel_plan = []

        for i in range(0,point_num):
            out_vel_plan.append(self.car_max_speed)

        for i in range(point_num, len(gloabl_path.poses) - point_num):
            x_list = []
            y_list = []
            for box in range(-point_num, point_num):
                x = gloabl_path.poses[i+box].pose.position.x
                y = gloabl_path.poses[i+box].pose.position.y
                x_list.append([-2*x, -2*y ,1])
                y_list.append((-x*x) - (y*y))

            x_matrix = np.array(x_list)
            y_matrix = np.array(y_list)
            x_trans = x_matrix.T


            a_matrix = np.linalg.inv(x_trans.dot(x_matrix)).dot(x_trans).dot(y_matrix)
            a = a_matrix[0]
            b = a_matrix[1]
            c = a_matrix[2]
            r = sqrt(a*a+b*b-c)
            v_max = sqrt(r*9.8*self.road_friction)
            if v_max > self.car_max_speed:
                v_max = self.car_max_speed
            out_vel_plan.append(v_max)

        for i in range(len(gloabl_path.poses) - point_num, len(gloabl_path.poses)-10):
            out_vel_plan.append(30)

        for i in range(len(gloabl_path.poses) - 10, len(gloabl_path.poses)):
            out_vel_plan.append(0)

        return out_vel_plan

class pidControl:
    def __init__(self):
        self.p_gain = 0.3
        self.i_gain = 0.07
        self.d_gain = 0.03
        self.prev_error = 0
        self.i_control = 0
        self.controlTime = 0.02

    def pid(self,target_vel, current_vel):
        error = target_vel - current_vel
        p_control = self.p_gain * error
        if error <= 5:
            self.i_control += self.i_gain * error * self.controlTime
        d_control = self.d_gain * (error-self.prev_error) / self.controlTime
        output = p_control + self.i_control + d_control
        self.prev_error = error
        return output

class vaildObject:
    
    def calc_vaild_obj(self,status_msg,object_data):
        
        self.all_object = object_data        
        ego_pose_x = status_msg.position.x
        ego_pose_y = status_msg.position.y
        ego_heading = status_msg.heading/180*pi
        
        global_npc_info = []
        local_npc_info  = []
        global_ped_info = []
        local_ped_info  = []
        global_obs_info = []
        local_obs_info  = []

        num_of_object = self.all_object.num_of_npcs + self.all_object.num_of_obstacle + self.all_object.num_of_pedestrian        
        if num_of_object > 0:

            #translation
            tmp_theta=ego_heading
            tmp_translation=[ego_pose_x, ego_pose_y]
            tmp_t=np.array([[cos(tmp_theta), -sin(tmp_theta), tmp_translation[0]],
                            [sin(tmp_theta),  cos(tmp_theta), tmp_translation[1]],
                            [0             ,               0,                  1]])
            tmp_det_t=np.array([[tmp_t[0][0], tmp_t[1][0], -(tmp_t[0][0] * tmp_translation[0] + tmp_t[1][0]*tmp_translation[1])],
                                [tmp_t[0][1], tmp_t[1][1], -(tmp_t[0][1] * tmp_translation[0] + tmp_t[1][1]*tmp_translation[1])],
                                [0,0,1]])

            #npc vehicle ranslation        
            for npc_list in self.all_object.npc_list:
                global_result=np.array([[npc_list.position.x],[npc_list.position.y],[1]])
                local_result=tmp_det_t.dot(global_result)
                if local_result[0][0]> 0 :        
                    global_npc_info.append([npc_list.type,npc_list.position.x,npc_list.position.y,npc_list.velocity.x])
                    local_npc_info.append([npc_list.type,local_result[0][0],local_result[1][0],npc_list.velocity.x])

            #ped translation
            for ped_list in self.all_object.pedestrian_list:
                global_result=np.array([[ped_list.position.x],[ped_list.position.y],[1]])
                local_result=tmp_det_t.dot(global_result)
                if local_result[0][0]> 0 :
                    global_ped_info.append([ped_list.type,ped_list.position.x,ped_list.position.y,ped_list.velocity.x])
                    local_ped_info.append([ped_list.type,local_result[0][0],local_result[1][0],ped_list.velocity.x])

            #obs translation
            for obs_list in self.all_object.obstacle_list:
                global_result=np.array([[obs_list.position.x],[obs_list.position.y],[1]])
                local_result=tmp_det_t.dot(global_result)
                if local_result[0][0]> 0 :
                    global_obs_info.append([obs_list.type,obs_list.position.x,obs_list.position.y,obs_list.velocity.x])
                    local_obs_info.append([obs_list.type,local_result[0][0],local_result[1][0],obs_list.velocity.x])
                
        return global_npc_info, local_npc_info, global_ped_info, local_ped_info, global_obs_info,local_obs_info

class cruiseControl: ## ACC(advanced cruise control) 적용 ##
    def __init__(self,object_vel_gain,object_dis_gain):
        self.npc_vehicle=[False,0]
        self.object=[False,0]
        self.Person=[False,0]
        self.object_vel_gain=object_vel_gain
        self.object_dis_gain=object_dis_gain

    ## 경로상의 장애물 유무 확인 (차량, 사람, obstacle) ##
    def checkObject(self,ref_path,global_npc_info, local_npc_info, 
                                  global_ped_info, local_ped_info, 
                                  global_obs_info, local_obs_info):

        min_rel_distance=float('inf')
        if len(global_ped_info) > 0 :        
            for i in range(len(global_ped_info)):
                for path in ref_path.poses :      
                    if global_ped_info[i][0] == 0 : # type=0 [pedestrian]                    
                        dis=sqrt(pow(path.pose.position.x-global_ped_info[i][1],2)+pow(path.pose.position.y-global_ped_info[i][2],2))
                        if dis<2.35:                            
                            rel_distance= sqrt(pow(local_ped_info[i][1],2)+pow(local_ped_info[i][2],2))
                            if rel_distance < min_rel_distance:
                                min_rel_distance=rel_distance
                                self.Person=[True,i]

        if len(global_npc_info) > 0 :            
            for i in range(len(global_npc_info)):
                for path in ref_path.poses :      
                    if global_npc_info[i][0] == 1 : # type=1 [npc_vehicle] 
                        dis=sqrt(pow(path.pose.position.x-global_npc_info[i][1],2)+pow(path.pose.position.y-global_npc_info[i][2],2))
                        if dis<2.35:
                            rel_distance= sqrt(pow(local_npc_info[i][1],2)+pow(local_npc_info[i][2],2))                            
                            if rel_distance < min_rel_distance:
                                min_rel_distance=rel_distance
                                self.npc_vehicle=[True,i]

        if len(global_obs_info) > 0 :            
            for i in range(len(global_obs_info)):
                for path in ref_path.poses :      
                    if global_obs_info[i][0] == 2 : # type=1 [obstacle] 
                        dis=sqrt(pow(path.pose.position.x-global_obs_info[i][1],2)+pow(path.pose.position.y-global_obs_info[i][2],2))
                        if dis<2.35:
                            rel_distance= sqrt(pow(local_obs_info[i][1],2)+pow(local_obs_info[i][2],2))                            
                            if rel_distance < min_rel_distance:
                                min_rel_distance=rel_distance
                                # self.object=[True,i]                    
                            

    # advanced cruise control 를 이용한 속도 계획 ##
    def acc(self,local_npc_info, local_ped_info, local_obs_info, ego_vel, target_vel): 
        out_vel =  target_vel
        default_space = 8
        time_gap = 0.8
        v_gain = self.object_vel_gain
        x_errgain = self.object_dis_gain

        if self.npc_vehicle[0]: #ACC ON_vehicle   
            print("ACC ON NPC_Vehicle")         
            front_vehicle = [local_npc_info[self.npc_vehicle[1]][1], local_npc_info[self.npc_vehicle[1]][2], local_npc_info[self.npc_vehicle[1]][3]]
            
            dis_safe = ego_vel * time_gap + default_space
            dis_rel = sqrt(pow(front_vehicle[0],2) + pow(front_vehicle[1],2))            
            vel_rel=((front_vehicle[2] / 3.6) - ego_vel)                        
            acceleration = vel_rel * v_gain - x_errgain * (dis_safe - dis_rel)

            out_vel = ego_vel + acceleration      

        if self.Person[0]: #ACC ON_Pedestrian
            print("ACC ON Pedestrian")
            Pedestrian = [local_ped_info[self.Person[1]][1], local_ped_info[self.Person[1]][2], local_ped_info[self.Person[1]][3]]
            
            dis_safe = ego_vel* time_gap + default_space
            dis_rel = sqrt(pow(Pedestrian[0],2) + pow(Pedestrian[1],2))            
            vel_rel = (Pedestrian[2] - ego_vel)              
            acceleration = vel_rel * v_gain - x_errgain * (dis_safe - dis_rel)    

            out_vel = ego_vel + acceleration
   
        if self.object[0]: #ACC ON_obstacle     
            print("ACC ON Obstacle")                    
            Obstacle = [local_obs_info[self.object[1]][1], local_obs_info[self.object[1]][2], local_obs_info[self.object[1]][3]]
            
            dis_safe = ego_vel* time_gap + default_space
            dis_rel = sqrt(pow(Obstacle[0],2) + pow(Obstacle[1],2))            
            vel_rel = (Obstacle[2] - ego_vel)
            acceleration = vel_rel * v_gain - x_errgain * (dis_safe - dis_rel)    

            out_vel = ego_vel + acceleration           

        return out_vel

class latticePlanner:

    def latticePlanner(self,ref_path,vehicle_status):
        out_path = []
        vehicle_pose_x = vehicle_status.position.x
        vehicle_pose_y = vehicle_status.position.y
        vehicle_velocity = vehicle_status.velocity.x*3.6

        look_distance=int(vehicle_velocity*0.2*2)

        if look_distance < 10 : #min 10m   
            look_distance = 10            
        elif look_distance > 20:
            look_distance = 20

        if len(ref_path.poses) > look_distance :

            global_ref_start_point = (ref_path.poses[0].pose.position.x, ref_path.poses[0].pose.position.y)
            global_ref_start_next_point = (ref_path.poses[1].pose.position.x, ref_path.poses[1].pose.position.y)

            global_ref_end_point = (ref_path.poses[look_distance*2].pose.position.x, ref_path.poses[look_distance*2].pose.position.y)
            
            theta = atan2(global_ref_start_next_point[1] - global_ref_start_point[1], global_ref_start_next_point[0] - global_ref_start_point[0])
            translation = [global_ref_start_point[0], global_ref_start_point[1]]

            t = np.array([[cos(theta), -sin(theta), translation[0]], [sin(theta), cos(theta), translation[1]], [0, 0, 1]])
            det_t = np.array([[t[0][0], t[1][0], -(t[0][0] * translation[0] + t[1][0] * translation[1])], [t[0][1], t[1][1], -(t[0][1] * translation[0] + t[1][1] * translation[1])],[0, 0, 1]])

            world_end_point = np.array([[global_ref_end_point[0]], [global_ref_end_point[1]], [1]])
            local_end_point = det_t.dot(world_end_point)
            world_ego_vehicle_position = np.array([[vehicle_pose_x], [vehicle_pose_y], [1]])
            local_ego_vehicle_position = det_t.dot(world_ego_vehicle_position)
            lane_off_set = [-3.0, -1.75, -1, 1, 1.75, 3.0]
            local_lattice_points = []
            
            for i in range(len(lane_off_set)):
                local_lattice_points.append([local_end_point[0][0], local_end_point[1][0] + lane_off_set[i], 1])
                
            for end_point in local_lattice_points :
                lattice_path = Path()
                lattice_path.header.frame_id = 'map'
                x = []
                y = []
                x_interval = 0.5
                xs = 0
                xf = end_point[0]
                ps = local_ego_vehicle_position[1][0]

                pf = end_point[1]
                x_num = xf / x_interval

                for i in range(xs,int(x_num)) : 
                    x.append(i*x_interval)
                
                a = [0.0, 0.0, 0.0, 0.0]
                a[0] = ps
                a[1] = 0
                a[2] = 3.0 * (pf - ps) / (xf * xf)
                a[3] = -2.0 * (pf - ps) / (xf * xf * xf)

                for i in x:
                    result = a[3] * i * i * i + a[2] * i * i + a[1] * i + a[0]
                    y.append(result)

                for i in range(0,len(y)) :
                    local_result = np.array([[x[i]], [y[i]], [1]])
                    global_result = t.dot(local_result)

                    read_pose = PoseStamped()
                    read_pose.pose.position.x = global_result[0][0]
                    read_pose.pose.position.y = global_result[1][0]
                    read_pose.pose.position.z = 0
                    read_pose.pose.orientation.x = 0
                    read_pose.pose.orientation.y = 0
                    read_pose.pose.orientation.z = 0
                    read_pose.pose.orientation.w = 1
                    lattice_path.poses.append(read_pose)

                out_path.append(lattice_path)

            #Add_point            
            add_point_size = min(int(vehicle_velocity * 2), len(ref_path.poses) )           
            
            for i in range(look_distance*2,add_point_size):
                if i+1 < len(ref_path.poses):
                    tmp_theta = atan2(ref_path.poses[i + 1].pose.position.y - ref_path.poses[i].pose.position.y,ref_path.poses[i + 1].pose.position.x - ref_path.poses[i].pose.position.x)                    
                    tmp_translation = [ref_path.poses[i].pose.position.x,ref_path.poses[i].pose.position.y]
                    tmp_t = np.array([[cos(tmp_theta), -sin(tmp_theta), tmp_translation[0]], [sin(tmp_theta), cos(tmp_theta), tmp_translation[1]], [0, 0, 1]])

                    for lane_num in range(len(lane_off_set)) :
                        local_result = np.array([[0], [lane_off_set[lane_num]], [1]])
                        global_result = tmp_t.dot(local_result)

                        read_pose = PoseStamped()
                        read_pose.pose.position.x = global_result[0][0]
                        read_pose.pose.position.y = global_result[1][0]
                        read_pose.pose.position.z = 0
                        read_pose.pose.orientation.x = 0
                        read_pose.pose.orientation.y = 0
                        read_pose.pose.orientation.z = 0
                        read_pose.pose.orientation.w = 1
                        out_path[lane_num].poses.append(read_pose)
            
            for i in range(len(out_path)):          
                globals()['lattice_pub_{}'.format(i+1)] = rospy.Publisher('/lattice_path_{}'.format(i+1),Path,queue_size=1)
                globals()['lattice_pub_{}'.format(i+1)].publish(out_path[i])
        
        return out_path

    def checkObject(self,ref_path,global_obs,lattice_path):        
        check_lens = len(lattice_path[0].poses)
        lattice = False
        for i in range(len(global_obs)):
            for index, path in enumerate(ref_path.poses) :   
                if index >= check_lens:
                    break
                else:
                    dis=sqrt(pow(path.pose.position.x-global_obs[i][1],2)+pow(path.pose.position.y-global_obs[i][2],2))
                    if dis<2.35:
                        lattice = True
                        break
        return lattice

    def collision_check(self,global_vaild_object,out_path):
        selected_lane = -1        
        lane_weight = [3,2,1,1,2,3] #reference path 
        
        for obj in global_vaild_object :                        
            for path_num in range(len(out_path)) :                    
                for path_pos in out_path[path_num].poses :                                
                    dis = sqrt(pow(obj[1] - path_pos.pose.position.x, 2) + pow(obj[2]-path_pos.pose.position.y, 2))

                    if dis < 1.5:
                        lane_weight[path_num] = lane_weight[path_num] + 100

        selected_lane = lane_weight.index(min(lane_weight))                    
        return selected_lane




if __name__ == '__main__':
    try:
        test_track=pure_pursuit()
    except rospy.ROSInterruptException:
        pass
