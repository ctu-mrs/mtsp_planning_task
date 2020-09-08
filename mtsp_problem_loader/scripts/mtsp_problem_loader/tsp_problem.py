"""
Custom TSP Loader
@author: R.Penicka
"""

import os
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import csv
import yaml


class MTSPProblem():
    """ class for storing .tsp file """

    KEY_VAL_DELIMITER = ":"
    NAME_PARAM = "NAME"
    TYPE_PARAM = "TYPE"
    COMMENT_PARAM = "COMMENT"
    DIMENSION_PARAM = "DIMENSION"
    NUMBER_OF_ROBOTS = "NUMBER_OF_ROBOTS"
    START_TARGET_INDEXES = "START_TARGET_INDEXES"
    NEIGHBORHOOD_RADIUS_PARAM = "NEIGHBORHOOD_RADIUS"
    EDGE_WEIGHT_TYPE_PARAM = "EDGE_WEIGHT_TYPE"
    NODE_COORD_SECTION_PARAM = "NODE_COORD_SECTION"
    EOF_PARAM = "EOF"

    # #{ __init__()

    def __init__(self, name, type, comment, dimension, number_of_robots, start_positions, neighborhood_radius, edge_weight_type, targets):
        self.name = name
        self.type = type
        self.comment = comment
        self.dimension = dimension
        self.number_of_robots = number_of_robots
        self.start_positions = start_positions
        self.neighborhood_radius = neighborhood_radius
        self.edge_weight_type = edge_weight_type
        self.targets = targets

    # #} end of __init__()

    # #{ __str__()

    def __str__(self):
        torenturn = ""
        for param in dir(self):
            if not param.startswith("__") and not param.endswith("__"):
                torenturn += ("problem.%s = %r" + os.linesep) % (param, getattr(self, param))
        return torenturn

    # #} end of __str__()

    # #{ get_param_from_line()

    @staticmethod
    def get_param_from_line(param, line, type=str):
        """ parse and change type of variable on .tsp file line """
        splited = line.split(MTSPProblem.KEY_VAL_DELIMITER)
        if len(splited) == 2:
            return type(splited[1].strip())
        else:
            print("error loading", param, "from line", line)
            quit()

    # #} end of get_param_from_line()

    # #{ load_problem()

    @staticmethod
    def load_problem(filename):
        """ load .tsp file to MTSPProblem proble """
        loaded_problem = {}
        loading_coords = False
        name = None
        type = None
        comment = None
        dimension = None
        number_of_robots = None
        neighborhood_radius = None
        edge_weight_type = None
        targets = None
        start_indexes = None
        with open(filename, 'r') as f:
            for line in f:
                if MTSPProblem.NAME_PARAM in line:
                    name = MTSPProblem.get_param_from_line(MTSPProblem.NAME_PARAM, line)
                if MTSPProblem.TYPE_PARAM in line:
                    type = MTSPProblem.get_param_from_line(MTSPProblem.TYPE_PARAM, line)
                if MTSPProblem.COMMENT_PARAM in line:
                    comment = MTSPProblem.get_param_from_line(MTSPProblem.COMMENT_PARAM, line)
                if MTSPProblem.DIMENSION_PARAM in line:
                    dimension = MTSPProblem.get_param_from_line(MTSPProblem.DIMENSION_PARAM, line, type=int)
                if MTSPProblem.NUMBER_OF_ROBOTS in line:
                    number_of_robots = MTSPProblem.get_param_from_line(MTSPProblem.NUMBER_OF_ROBOTS, line, type=int)
                if MTSPProblem.START_TARGET_INDEXES in line: 
                    start_indexes_str = MTSPProblem.get_param_from_line(MTSPProblem.START_TARGET_INDEXES, line, type=str)
                    start_indexes_str_list = start_indexes_str.split(",")
                    start_indexes = [int(i) for i in start_indexes_str_list]
                if MTSPProblem.NEIGHBORHOOD_RADIUS_PARAM in line:
                    neighborhood_radius = MTSPProblem.get_param_from_line(MTSPProblem.NEIGHBORHOOD_RADIUS_PARAM, line, type=float)
                if MTSPProblem.EDGE_WEIGHT_TYPE_PARAM in line:
                    edge_weight_type = MTSPProblem.get_param_from_line(MTSPProblem.EDGE_WEIGHT_TYPE_PARAM, line)
                if MTSPProblem.NODE_COORD_SECTION_PARAM in line:
                    loading_coords = True
                    targets = []
                    continue

                if MTSPProblem.EOF_PARAM in line:
                    targets_only = []
                    start_positions = []
                    for target in targets:
                        if target[0] in start_indexes:
                            start_positions.append(target)
                        else:
                            targets_only.append(target)
                    return MTSPProblem(name, type, comment, dimension, number_of_robots, start_positions, neighborhood_radius, edge_weight_type, targets_only)

                if loading_coords:
                    splited = line.split()  # 1 2.9 -2.4
                    if len(splited) == 3:
                        targets.append([int(splited[0]), float(splited[1]), float(splited[2])])
                    else:
                        print("error loading", MTSPProblem.NODE_COORD_SECTION_PARAM, "from line", line)
                        quit()

    # #} end of load_problem()

    # #{ save_problem()

    @staticmethod
    def save_problem(filename, name, type, comment, edge_weight_type, targets, num_robots, start_indexes, neighborhood_radius):
        """ save problem to .tsp file """
        with open(filename, 'w') as f:
            f.write(MTSPProblem.NAME_PARAM + MTSPProblem.KEY_VAL_DELIMITER + " " + name + os.linesep)
            f.write(MTSPProblem.TYPE_PARAM + MTSPProblem.KEY_VAL_DELIMITER + " " + type + os.linesep)
            f.write(MTSPProblem.COMMENT_PARAM + MTSPProblem.KEY_VAL_DELIMITER + " " + comment + os.linesep)
            f.write(MTSPProblem.DIMENSION_PARAM + MTSPProblem.KEY_VAL_DELIMITER + " " + str(len(targets)) + os.linesep)
            f.write(MTSPProblem.NUMBER_OF_ROBOTS + MTSPProblem.KEY_VAL_DELIMITER + " " + str(num_robots) + os.linesep)
            f.write(MTSPProblem.START_TARGET_INDEXES + MTSPProblem.KEY_VAL_DELIMITER + " " + ", ".join([str(i + 1) for i in start_indexes]) + os.linesep)
            f.write(MTSPProblem.NEIGHBORHOOD_RADIUS_PARAM + MTSPProblem.KEY_VAL_DELIMITER + " " + str(neighborhood_radius) + os.linesep)
            f.write(MTSPProblem.EDGE_WEIGHT_TYPE_PARAM + MTSPProblem.KEY_VAL_DELIMITER + " " + edge_weight_type + os.linesep)

            f.write(MTSPProblem.NODE_COORD_SECTION_PARAM + os.linesep)

            for i in range(len(targets)):
                f.write(str(i + 1) + " " + str(targets[i][0]) + " " + str(targets[i][1]) + os.linesep)

            f.write(MTSPProblem.EOF_PARAM + os.linesep)

    # #} end of save_problem()

    # #{ plot_problem()

    @staticmethod
    def plot_problem(mtsp_problem, show=True):
        """ plot MTSPProblem proble using matplotlib"""
        figsize = (10, 7)
        fig = plt.figure(figsize=figsize)
        ax = fig.gca()
        ax.set_ylabel('y [m]')
        ax.set_xlabel('x [m]')
        plt.axis('equal')
        plt.plot([item[1] for item in mtsp_problem.targets], [item[2] for item in mtsp_problem.targets], 'k.')
        for item in mtsp_problem.targets:
            circle = mpatches.Circle(item[1:3], mtsp_problem.neighborhood_radius, edgecolor='k', alpha=0.2)
            ax.add_artist(circle)
        for item in mtsp_problem.start_positions:
            plt.plot([item[1]], [item[2]], 'go')
        if show:
            plt.show()
        return ax

    # #} end of plot_problem()

    
def load_gazebo_uav_init_file(filename):
    with open(filename, 'r') as csvfile:
        csv_rows = csv.reader(csvfile)
        for row in csv_rows:
            return [float(row[1]), float(row[2])]


def read_world_file_arena(filename):
    arena_corners = []
    with open(filename, 'r') as f:
        data_loaded = yaml.safe_load(f)
        for idx in range(len(data_loaded['safety_area']['safety_area'])):
            loc = data_loaded['safety_area']['safety_area'][idx]
            if idx % 2 == 0:
                arena_corners.append([])
            arena_corners[-1].append(loc)
        return arena_corners   
