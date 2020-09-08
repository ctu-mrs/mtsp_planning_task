#!/usr/bin/env python3

import random, math
from tsp_problem import MTSPProblem, load_gazebo_uav_init_file , read_world_file_arena
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon


NUM_TARGETS = 20
NUM_VEHICLES = 2
NUM_AFTER_DECIMAL_POINTS = 1

UAV_START_LOCATIONS = [[ 15, -21],
                       [ -5, -25]]

filename = "../../problems/random_problem.tsp"
name = "random problem"
type = "MTSP"
comment = "MTSP problem for MRS UAV summer school"
edge_weight_type =  "EUC_2D"
num_robots = 2
neighborhood_radius = 2 

arena_file = "../../../mtsp_state_machine/config/simulation/world.yaml"
uav_start_pos_files = ["../../../mtsp_state_machine/tmux/test/pos1.txt", "../../../mtsp_state_machine/tmux/test/pos2.txt"]
#fill targets array with random distinct targets within x_limits adn y_limits bounds




uav_start_positions = []
for i in range(num_robots):
    pos = load_gazebo_uav_init_file(uav_start_pos_files[i])
    uav_start_positions.append(pos)
    print(pos)

arena_corners = read_world_file_arena(arena_file)
print("arena_corners",arena_corners)
arena_polygon = Polygon(arena_corners)

x_limits = [min(p[0] for p in arena_corners), max(p[0] for p in arena_corners)]
y_limits = [min(p[1] for p in arena_corners), max(p[1] for p in arena_corners)]


targets = []
targets += uav_start_positions
start_indexes = [i for i in range(len(uav_start_positions))]
while len(targets) < NUM_TARGETS:
    x = random.randint(x_limits[0]*math.pow(10, NUM_AFTER_DECIMAL_POINTS),x_limits[1]*math.pow(10, NUM_AFTER_DECIMAL_POINTS))/float(math.pow(10, NUM_AFTER_DECIMAL_POINTS))
    y = random.randint(y_limits[0]*math.pow(10, NUM_AFTER_DECIMAL_POINTS),y_limits[1]*math.pow(10, NUM_AFTER_DECIMAL_POINTS))/float(math.pow(10, NUM_AFTER_DECIMAL_POINTS))
    point = Point(x,y)

    if arena_polygon.exterior.distance(point) < neighborhood_radius:
        continue
    
    if not arena_polygon.contains(point):
        continue
    
    target = [x,y]
    if target not in targets:
        targets.append(target)
    
    
#save random problem to filename file
MTSPProblem.save_problem(filename, name, type, comment, edge_weight_type, targets, num_robots,start_indexes, neighborhood_radius)

#plot the randomly generated problem
figsize = (12,8)
fig = plt.figure(figsize=figsize)
ax = fig.gca()
ax.set_ylabel('x [m]')
ax.set_xlabel('y [m]')
plt.axis('equal')
plt.plot([item[0] for item in targets],[item[1] for item in targets],'k.')
for i in range(len(targets)):
    item = targets[i]
    if i not in start_indexes:
        circle = mpatches.Circle(item, neighborhood_radius)
        ax.add_artist(circle)
    else:
        plt.plot([item[0]],[item[1]],'go')
ax, ay = arena_polygon.exterior.xy
plt.plot(ax,ay,'r-')
plt.show()

