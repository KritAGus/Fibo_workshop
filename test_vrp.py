#!/usr/bin/python3
from networkx import DiGraph, from_numpy_matrix, relabel_nodes, set_node_attributes
import networkx as nx
from networkx import from_numpy_matrix, relabel_nodes, set_node_attributes
from vrpy import VehicleRoutingProblem
from numpy import array

# Distance matrix
DISTANCES = [
[0,548,776,696,582,274,502,194,308,194,536,502,388,354,468,776,662,0], # from Source
[0,0,684,308,194,502,730,354,696,742,1084,594,480,674,1016,868,1210,548],
[0,684,0,992,878,502,274,810,468,742,400,1278,1164,1130,788,1552,754,776],
[0,308,992,0,114,650,878,502,844,890,1232,514,628,822,1164,560,1358,696],
[0,194,878,114,0,536,764,388,730,776,1118,400,514,708,1050,674,1244,582],
[0,502,502,650,536,0,228,308,194,240,582,776,662,628,514,1050,708,274],
[0,730,274,878,764,228,0,536,194,468,354,1004,890,856,514,1278,480,502],
[0,354,810,502,388,308,536,0,342,388,730,468,354,320,662,742,856,194],
[0,696,468,844,730,194,194,342,0,274,388,810,696,662,320,1084,514,308],
[0,742,742,890,776,240,468,388,274,0,342,536,422,388,274,810,468,194],
[0,1084,400,1232,1118,582,354,730,388,342,0,878,764,730,388,1152,354,536],
[0,594,1278,514,400,776,1004,468,810,536,878,0,114,308,650,274,844,502],
[0,480,1164,628,514,662,890,354,696,422,764,114,0,194,536,388,730,388],
[0,674,1130,822,708,628,856,320,662,388,730,308,194,0,342,422,536,354],
[0,1016,788,1164,1050,514,514,662,320,274,388,650,536,342,0,764,194,468],
[0,868,1552,560,674,1050,1278,742,1084,810,1152,274,388,422,764,0,798,776],
[0,1210,754,1358,1244,708,480,856,514,468,354,844,730,536,194,798,0,662],
[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], # from Sink
]


# The matrix is transformed into a DiGraph
A = array(DISTANCES, dtype=[("cost", int)])
G = from_numpy_matrix(A, create_using=nx.DiGraph())

# Demands (key: node, value: amount)
DEMAND = {1: 1, 2: 1, 3: 2, 4: 4, 5: 2, 6: 4, 7: 8, 8: 8, 9: 1, 10: 2, 11: 1, 12: 2, 13: 4, 14: 4, 15: 8, 16: 8}
# The demands are stored as node attributes
set_node_attributes(G, values=DEMAND, name="demand")

# The depot is relabeled as Source and Sink
G = relabel_nodes(G, {0: "Source", 17: "Sink"})
pickup_deliveries = {(1,6): 1,
                    (2,10): 2,
                    (4,3): 3,
                    (5,9): 1,
                    (7,8): 2,
                    (15,11): 3,
                    (13,12): 1,
                    (16,14): 4
                    }
prob = VehicleRoutingProblem(G,load_capacity=6,num_stops=6, pickup_delivery=True)

for (u,v) in pickup_deliveries:
    G.nodes[u]['request'] = v
    G.nodes[u]['demand'] = pickup_deliveries[(u,v)]
    G.nodes[v]['demand'] = -pickup_deliveries[(u,v)]
prob.solve(cspy=False)
print(prob.best_routes)