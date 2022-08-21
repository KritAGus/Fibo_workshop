#!/usr/bin/python3
from errno import EDEADLK
from turtle import distance, pos, st
import networkx as nx
from ament_index_python.packages import get_package_share_directory
import os, yaml, math
import matplotlib.pyplot as plt


def dist(pos,e):
    math.sqrt((pos[e[0]][0]-pos[e[1]][0])**2+(pos[e[0]][1]-pos[e[1]][1])**2)

multi_turtlesim_traffic_path = get_package_share_directory('multi_turtlesim_traffic')
traffic_path = os.path.join(multi_turtlesim_traffic_path,'config','traffic.yaml')

with open(traffic_path) as f:
    traffic_info = yaml.load(f, Loader=yaml.loader.SafeLoader)

g = nx.Graph()
id = 0
for node in traffic_info['vertices']:
    g.add_node(id, pos=node)
    id = id+1
pos = nx.get_node_attributes(g,'pos')
for edge in traffic_info['edges']:
    g.add_edge(edge[0],edge[1])
    nx.set_edge_attributes(g,{e: dist(pos,e) for e in g.edges()},"cost")
# print(g)
nx.draw(g, pos=pos)
print(nx.shortest_path(g,8,4))


idi = 0
num_stations = len(traffic_info['stations'])+1
distance_matrix = [[0]*num_stations for i in range(num_stations)]
for station_i in traffic_info['stations']:
    idj = 0
    for station_j in traffic_info['stations']:
        distance_matrix[idi][idj] = nx.shortest_path_length(g,station_i,station_j)
        idj+=1
    distance_matrix[idi][idj] = distance_matrix[idi][0]
    distance_matrix[idi][0] = 0
    idi+=1

print(distance_matrix)
plt.show()
    
# print(nx.get_node_attributes(g,'pos'))