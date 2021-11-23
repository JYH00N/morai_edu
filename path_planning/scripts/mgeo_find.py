#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import rospy

from math import pi,pow,sqrt
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped,Point32,PoseWithCovarianceStamped
from sensor_msgs.msg import PointCloud

current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_path)

from lib.mgeo.class_defs import *
from e_dijkstra import Dijkstra

# load_path = os.path.normpath(os.path.join(current_path, 'lib/mgeo_data/kcity'))
# mgeo_planner_map = MGeoPlannerMap.create_instance_from_json(load_path)

# node_set = mgeo_planner_map.node_set
# link_set = mgeo_planner_map.link_set
# nodes=node_set.nodes
# links=link_set.lines

class find_mgeo_data :

    def __init__(self,mgeo_map):
        
        self.map_name = mgeo_map

        load_path = os.path.normpath(os.path.join(current_path, 'lib/mgeo_data/'+self.map_name))
        mgeo_planner_map = MGeoPlannerMap.create_instance_from_json(load_path)

        node_set = mgeo_planner_map.node_set
        link_set = mgeo_planner_map.link_set
        self.nodes=node_set.nodes
        self.links=link_set.lines
        self.link_msg=self.getAllLinks()
        self.node_msg=self.getAllNode()
        self.global_planner=Dijkstra(self.nodes,self.links)

    def getAllLinks(self):
        all_link=PointCloud()
        all_link.header.frame_id='map'
        
        for link_idx in self.links :
            for link_point in self.links[link_idx].points:
                tmp_point=Point32()
                tmp_point.x=link_point[0]
                tmp_point.y=link_point[1]
                tmp_point.z=link_point[2]
                all_link.points.append(tmp_point)

        return all_link
    
    def getAllNode(self):
        all_node=PointCloud()
        all_node.header.frame_id='map'
        for node_idx in self.nodes :
            tmp_point=Point32()
            tmp_point.x=self.nodes[node_idx].point[0]
            tmp_point.y=self.nodes[node_idx].point[1]
            tmp_point.z=self.nodes[node_idx].point[2]
            all_node.points.append(tmp_point)

        return all_node

    def ego_status(self,ego_msg):
        self.x = ego_msg.position.x
        self.y = ego_msg.position.y
        self.vel = ego_msg.velocity.x

    def goal_position(self,goal_msg):
        self.destination_x=goal_msg.pose.position.x
        self.destination_y=goal_msg.pose.position.y

    def find_current_ego_link(self):
        min_dis = float('inf')
        self.idx = None
        self.current_idx = None
        for link_idx in self.links :
            for link_point in self.links[link_idx].points:                
                x = link_point[0]
                y = link_point[1]

                dx = self.x - x
                dy = self.y - y

                dist = sqrt(pow(dx,2)+pow(dy,2))         
                if dist < min_dis:
                    min_dis = dist
                    self.idx = link_idx     
        
        return self.idx

    def find_link_idx(self,pose_x,pose_y):
        min_dis = float('inf')
        self.idx = None
        self.current_idx = None
        for link_idx in self.links :
            for link_point in self.links[link_idx].points:                
                x = link_point[0]
                y = link_point[1]

                dx = pose_x - x
                dy = pose_y - y

                dist = sqrt(pow(dx,2)+pow(dy,2))         
                if dist < min_dis:
                    min_dis = dist
                    self.idx = link_idx     
        
        return self.idx

    def check_lc_left(self):
        self.left_lc = False
        self.left_idx = None
        ego_link_idx = self.find_current_ego_link()
        if self.links[ego_link_idx].lane_ch_link_left != None:
            rospy.loginfo("can left line change")
            self.left_lc = True
            self.left_idx = self.links[ego_link_idx].lane_ch_link_left.idx
        return self.left_lc,self.left_idx

    def check_lc_right(self):
        self.right_lc = False
        self.right_idx = None
        ego_link_idx = self.find_current_ego_link()
        if self.links[ego_link_idx].lane_ch_link_right != None:
            rospy.loginfo("can right line change")
            self.right_lc = True
            self.right_idx = self.links[ego_link_idx].lane_ch_link_right.idx
        return self.right_lc,self.right_idx
        
    def calc_dijkstra_path(self):
        x = self.x
        y = self.y
        goal_x=self.destination_x
        goal_y=self.destination_y
        
        start_min_dis=float('inf')
        goal_min_dis=float('inf')


        for node_idx in self.nodes:
            node_pose_x=self.nodes[node_idx].point[0]
            node_pose_y=self.nodes[node_idx].point[1]
            start_dis=sqrt(pow(x-node_pose_x,2)+pow(y-node_pose_y,2))
            goal_dis=sqrt(pow(goal_x-node_pose_x,2)+pow(goal_y-node_pose_y,2))
            if start_dis < start_min_dis :
                start_min_dis=start_dis
                start_node_idx=node_idx
            if goal_dis < goal_min_dis :
                goal_min_dis=goal_dis
                end_node_idx=node_idx

        ego_link_idx = self.find_current_ego_link()
        start_node_idx = self.links[ego_link_idx].from_node.idx

        result, path = self.global_planner.find_shortest_path(start_node_idx, end_node_idx)
        dijkstra_path=Path()
        dijkstra_path.header.frame_id='map'
        for waypoint in path["point_path"] :
            tmp_point=PoseStamped()
            tmp_point.pose.position.x=waypoint[0]
            tmp_point.pose.position.y=waypoint[1]
            dijkstra_path.poses.append(tmp_point)
            # print(waypoint[0],waypoint[1])

        return dijkstra_path

    def calc_obj_dijkstra_path(self,pose_x,pose_y):
        x = self.x
        y = self.y
        goal_x=pose_x
        goal_y=pose_y
        # print(goal_x,goal_y)
        start_min_dis=float('inf')
        goal_min_dis=float('inf')


        for node_idx in self.nodes:
            node_pose_x=self.nodes[node_idx].point[0]
            node_pose_y=self.nodes[node_idx].point[1]
            start_dis=sqrt(pow(x-node_pose_x,2)+pow(y-node_pose_y,2))
            goal_dis=sqrt(pow(goal_x-node_pose_x,2)+pow(goal_y-node_pose_y,2))
            if start_dis < start_min_dis :
                start_min_dis=start_dis
                start_node_idx=node_idx
            if goal_dis < goal_min_dis :
                goal_min_dis=goal_dis
                end_node_idx=node_idx

        ego_link_idx = self.find_current_ego_link()
        start_node_idx = self.links[ego_link_idx].from_node.idx

        result, path = self.global_planner.find_shortest_path(start_node_idx, end_node_idx)
        dijkstra_path=Path()
        dijkstra_path.header.frame_id='map'
        for waypoint in path["point_path"] :
            tmp_point=PoseStamped()
            tmp_point.pose.position.x=waypoint[0]
            tmp_point.pose.position.y=waypoint[1]
            dijkstra_path.poses.append(tmp_point)
            # print(waypoint[0],waypoint[1])

        
        
        return dijkstra_path