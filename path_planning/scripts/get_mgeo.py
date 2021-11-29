#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys


current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_path)

from lib.mgeo.class_defs import *


load_path = os.path.normpath(os.path.join(current_path, 'lib/mgeo_data/kcity'))
mgeo_planner_map = MGeoPlannerMap.create_instance_from_json(load_path)

node_set = mgeo_planner_map.node_set
link_set = mgeo_planner_map.link_set
nodes=node_set.nodes
links=link_set.lines
print('# of nodes: ', len(node_set.nodes))
print('# of links: ', len(link_set.lines))
# print(link_set.lines)



# print(node_set.nodes.keys())  # 노드 이름들 출력
# print(link_set.lines.keys())

# print("idx",node_set.nodes['A219BS010644'])
# print("to_links",node_set.nodes['A219BS010644'].to_links)

## nodes - field
# print("nodes dictionary                     : ",node_set.nodes.items())
print("nodes_idx                            : ",node_set.nodes['A119BS010714'].idx)
print("nodes_to_links_idx                   : ",node_set.nodes['A119BS010714'].to_links[0].idx)
print("nodes_from_links_idx                 : ",node_set.nodes['A119BS010714'].from_links[0].idx)
print("nodes_on_stop_line                   : ",node_set.nodes['A119BS010227'].on_stop_line)
    
## nodes - method   
print("nodes_get_to_links                   : ",node_set.nodes['A119BS010714'].get_to_links())
print("nodes_get_from_links                 : ",node_set.nodes['A119BS010714'].get_from_links())
print("nodes_get_to_links_list              : ",node_set.nodes['A119BS010714'].get_to_links_idx_list())
print("nodes_get_from_links_list            : ",node_set.nodes['A119BS010714'].get_from_links_idx_list())
print("nodes_get_to_nodes                   : ",node_set.nodes['A119BS010714'].get_to_nodes())
print("nodes_get_from_nodes                 : ",node_set.nodes['A119BS010714'].get_from_nodes())
print("nodes_get_junctions                  : ",node_set.nodes['A119BS010714'].get_junctions())
print("nodes_get_junctions_list             : ",node_set.nodes['A119BS010714'].get_junctions_idx_list())
print("nodes_related_nodes_and_links        : ",node_set.nodes['A119BS010714'].print_all_related_nodes_and_links())
    
## links - field    
print("links_idx                            : ",link_set.lines['A219BS010419'].idx)
print("links_from_nodes_idx                 : ",link_set.lines['A219BS010419'].from_node.idx)
print("links_to_nodes_idx                   : ",link_set.lines['A219BS010419'].to_node.idx)
print("links_lazy_point_init                : ",link_set.lines['A219BS010419'].lazy_point_init)
print("links_lane_change_pair_list          : ",link_set.lines['A219BS010419'].lane_change_pair_list)
print("links_max_speed_kph                  : ",link_set.lines['A219BS010419'].max_speed_kph)
print("links_min_speed_kph                  : ",link_set.lines['A219BS010419'].min_speed_kph)
print("links_traffic_signs                  : ",link_set.lines['A219BS010419'].traffic_signs)
print("links_traffic_lights                 : ",link_set.lines['A219BS010419'].traffic_lights)
    
## links - method   
print("links_get_to_node                    : ",link_set.lines['A219BS010315'].get_to_node().idx)
print("links_get_from_node                  : ",link_set.lines['A219BS010315'].get_from_node().idx)
print("links_get_to_links                   : ",link_set.lines['A219BS010315'].get_to_links()[0].idx)
print("links_get_from_links                 : ",link_set.lines['A219BS010315'].get_from_links()[0].idx)
print("links_is_it_for_lane_change          : ",link_set.lines['A219BS010315'].is_it_for_lane_change())
print("links_get_number_of_lane_change      : ",link_set.lines['A219BS010315'].get_number_of_lane_change())

print("left                                 : ",link_set.lines['A219BS010420'].lane_ch_link_left.idx)

print("test")