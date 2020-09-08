"""
Various types of TSP
@author: P.Vana & P.Cizek & R.Penicka
"""

import os, sys
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from sklearn.cluster import KMeans  # need to install sudo apt-get install python3-sklearn python2-sklearn
import numpy as np
import math
import dubins
import tsp_trajectory
from tsp_trajectory import dist_euclidean, pos_in_distance

this_script_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(this_script_dir, "lkh"))
import invoke_LKH




class TSPSolver():

    def __init__(self):
        pass

    # #{ cluester_kmeans()

    def cluster_kmeans(self, targets, num_clusters):
        """ find k-means clustering into num_clusters clusters and return 2d array of cluster's targets and also cluster centers"""
        
        targets_xy = [[i[1], i[2]] for i in targets]
        targets_xy_array = np.array(targets_xy)
        km = KMeans(n_clusters=num_clusters)
        target_center_ids = km.fit_predict(targets_xy_array)
        cluster_centers = km.cluster_centers_
        clusters = []
        for i in range(num_clusters):
            clusters.append([])
            for target_id in range(len(target_center_ids)):
                if target_center_ids[target_id] == i:
                    clusters[-1].append(targets[target_id])

        return clusters, cluster_centers

    # #} end of cluester_kmeans()

    # #{ plan_tour_etsp()

    def plan_tour_etsp(self, targets, start_idx):
        """ solve euclidean tsp on targets with given goals and starts"""
        
        n = len(targets)
        self.distances = np.zeros((n, n))
        self.paths = {}

        # find path between each pair of goals (a,b)
        for a in range(0, n):
            for b in range(0, n):
                g1 = targets[a][1:3]
                g2 = targets[b][1:3]
                if a != b:
                    # store distance
                    self.distances[a][b] = dist_euclidean(g1, g2)
                    # store path
                    self.paths[(a, b)] = [g1, g2]

        return self.compute_tsp_tour(start_idx=start_idx)

    # #} end of plan_tour_etsp()

    # #{ compute_tsp_tour()

    def compute_tsp_tour(self, start_idx=None, end_idx=None):
        """compute the shortest tour based on the distance matrix (self.distances)"""
        
        n = len(self.distances)
        sequence = self.compute_tsp_sequence(start_idx=start_idx, end_idx=end_idx)

        if start_idx != None and end_idx != None:
            # TODO - reconstruct only the open-loop
            path = []
            for a in range(start_idx, start_idx + n - 1):
                b = (a + 1) % n
                a_idx = sequence[a]
                b_idx = sequence[b]
                actual_path = self.paths[(a_idx, b_idx)]
                path = path + actual_path
        else:
            path = []
            for a in range(0, n):
                b = (a + 1) % n
                a_idx = sequence[a]
                b_idx = sequence[b]
                actual_path = self.paths[(a_idx, b_idx)]
                path = path + actual_path

        return path

    # #} end of compute_tsp_tour()

    # #{ compute_tsp_sequence()

    def compute_tsp_sequence(self, start_idx=None, end_idx=None):
        """ compute the shortest sequence based on the distance matrix (self.distances) using LKH """

        n = len(self.distances)

        if start_idx != None and end_idx != None:
            # TODO - insert your code here
            print ("TODO")
            quit()
            M = n * np.max(np.max(self.distances))
            for i in range(n):
                self.distances[i, start_idx] = M
                self.distances[end_idx, i] = M
            self.distances[end_idx, start_idx] = 0

        fname_tsp = "problem"
        user_comment = "a comment by the user"
        invoke_LKH.writeTSPLIBfile_FE(fname_tsp, self.distances, user_comment)
        invoke_LKH.run_LKHsolver_cmd(fname_tsp)  # , silent=True)
        sequence = invoke_LKH.read_LKHresult_cmd(fname_tsp)
        
        if len(sequence) > 0 and sequence[0] != start_idx:
            for i in range(len(sequence)):
                if sequence[i] == start_idx:
                    new_sequence = sequence[i:len(sequence)] + sequence[0:i]
                    sequence = new_sequence
                    break
        return sequence

    # #} end of compute_tsp_sequence()

    # #{ plan_tour_etspn_decoupled()

    def plan_tour_etspn_decoupled(self, goals, start_idx, neighborhood_radius):
        """planning of euclidean tsp with neighborhoods"""
        
        return self.plan_tour_dtspn_decoupled(goals, start_idx, neighborhood_radius, 0)

    # #} end of plan_tour_etspn_decoupled()

    # #{ plan_tour_dtspn_decoupled()

    def plan_tour_dtspn_decoupled(self, goals, start_idx, sensing_radius, turning_radius):
        """planning of dubins tsp with neighborhoods"""
        
        n = len(goals)
        self.distances = np.zeros((n, n))
        self.paths = {}

        # find path between each pair of goals (a,b)
        for a in range(0, n):
            for b in range(0, n):
                g1 = goals[a][1:3]
                g2 = goals[b][1:3]
                if a != b:
                    # store distance
                    self.distances[a][b] = dist_euclidean(g1, g2)

        sequence = self.compute_tsp_sequence(start_idx=start_idx)

        samples_position = 8
        if sensing_radius == 0 :
            samples_position = 1

        samples_heading = 8
        if turning_radius == 0:
            samples_heading = 1

        samples = []
        for idx, g in enumerate(goals):
            samples.append([])
            if idx == start_idx:
                local_samples_position = 1
                local_sensing_radius = 0
            else:
                local_samples_position = samples_position
                local_sensing_radius = sensing_radius
                
            for sp in range(local_samples_position):
                for sh in range(samples_heading):
                    alpha = sp * 2 * math.pi / samples_heading
                    position = g[1:3] + local_sensing_radius * np.array([math.cos(alpha), math.sin(alpha)])
                    heading = sh * 2 * math.pi / samples_heading
                    sample = (position[0], position[1], heading)
                    samples[idx].append(sample)

        # TODO - this code select the first sample in the set!
        selected_samples = [0] * n

        distances = []

        for i in range(n):
            distances.append([])
            a = sequence[i]
            b = sequence[(i + 1) % n]
            # print('Shortest paths from city {} to {}'.format(a, b))

            n1 = samples[a]
            n2 = samples[b]
            for i1 in range(len(n1)):
                distances[i].append([])
                for i2 in range(len(n2)):
                    # print("Samples {} {}".format(i1, i2))
                    s1 = n1[i1]
                    s2 = n2[i2]
                    # path
                    if turning_radius == 0:
                        dist = dist_euclidean(s1[0:2], s2[0:2])
                    else:
                        dubins_path = dubins.shortest_path(s1, s2, turning_radius)
                        dist = dubins_path.path_length()

                    # store results
                    distances[i][i1].append(dist)

        best_len = sys.float_info.max
        best_tour = []

        no_start = len(distances[0])
        for start in range(no_start):
            # print('Try start', start)

            # shortest sh[region_idx][sample_idx]
            # contains (prev, length)
            sh = []
            sh.append([])
            for i_start in range(no_start):
                if i_start == start:
                    sh[0].append((-1, 0))
                else:
                    sh[0].append((-1, sys.float_info.max))

            for region_idx in range(n):
                # print("Through region", region_idx)

                n1 = len(distances[region_idx])
                n2 = len(distances[region_idx][0])

                sh.append([])
                for idx2 in range(n2):
                    s_len = sys.float_info.max
                    s_idx = -1
                    for idx1 in range(n1):
                        dst = sh[region_idx][idx1][1]
                        dst += distances[region_idx][idx1][idx2]
                        if dst < s_len:
                            s_len = dst
                            s_idx = idx1
                    sh[region_idx + 1].append((s_idx, s_len))

            act_sol = sh[-1][start]
            # print(act_sol)
            if act_sol[1] < best_len:
                best_len = act_sol[1]
                tour = []
                act = start
                for i in range(n):
                    act = sh[n - i][act][0]
                    tour.append(act)
                best_tour = list(reversed(tour))
                # print("New tour", best_tour)

        selected_samples = best_tour
        print(selected_samples)

        # create path from selected samples indexes
        path = []
        for a in range(0, n):
            b = (a + 1) % n
            g1 = goals[sequence[a]]
            g2 = goals[sequence[b]]

            start = samples[sequence[a]][selected_samples[a]]
            end = samples[sequence[b]][selected_samples[b]]
            # print("start",start)
            # print("end",end)
            step_size = 0.1
            if turning_radius == 0:
                if a == 0:
                    path.append(start[0:2])
                path.append(end[0:2])
            else:
                if a == 0:
                    path.append(start)
                path.append(end)

        print("plan_tour_dtspn_decoupled path", path)
        return path

    # #} end of plan_tour_dtspn_decoupled()
    
    # #{ plan_tour_dtspn_noon_bean()
    
    def plan_tour_dtspn_noon_bean(self, goals, start_idx, sensing_radius, turning_radius):
        n = len(goals)     
       
        samples_position = 4
        if sensing_radius == 0 :
            samples_position = 1

        samples_heading = 4
        if turning_radius == 0:
            samples_heading = 1

        k = samples_position * samples_heading
        
        samples = []
        sample_target = {}
        taget_samples = {}
        for idx, g in enumerate(goals):
            # samples.append([])
            if idx == start_idx:
                local_samples_position = 1
                local_sensing_radius = 0
            else:
                local_samples_position = samples_position
                local_sensing_radius = sensing_radius
            
            for sp in range(local_samples_position):
                for sh in range(samples_heading):
                    alpha = sp * 2 * math.pi / samples_heading
                    position = g[1:3] + local_sensing_radius * np.array([math.cos(alpha), math.sin(alpha)])
                    heading = sh * 2 * math.pi / samples_heading
                    sample = (position[0], position[1], heading)
                    
                    # save mapping from samples to targets and reversed
                    sample_target[len(samples)] = idx
                    if taget_samples.get(idx) is None:
                        taget_samples[idx] = []
                    taget_samples[idx].append(len(samples))
                    
                    # add sample
                    samples.append(sample)
                    print(sample)

        num_samples = len(samples)
        # matrix of all distances between samples of size[len(samples),len(samples)]
        all_distances = np.zeros([num_samples, num_samples])
        
        max_dist = 0
        for i in range(num_samples):
            for j in range(num_samples):
                # print('Shortest paths from city {} to {}'.format(i, j))
                s1 = samples[i]
                s2 = samples[j]
                # path
                if turning_radius == 0:
                    dist = dist_euclidean(s1[0:2], s2[0:2])
                else:
                    dubins_path = dubins.shortest_path(s1, s2, turning_radius)
                    dist = dubins_path.path_length()
                if dist > max_dist:
                    max_dist = dist
                # store results
                all_distances[i][j] = dist

        self.distances = np.zeros([num_samples, num_samples])    
        self.paths = {} 
        M = n * max_dist
        M2 = n * n * max_dist
        
        for i in range(num_samples):
            for j in range(num_samples):
                self.distances[i][j] = M2 
                
        for set_id in range(n):
            for idx_in_ts in range(len(taget_samples[set_id])):
                from_sample_i = taget_samples[set_id][idx_in_ts - 1]
                to_sample_i = taget_samples[set_id][idx_in_ts]
                self.distances[from_sample_i][to_sample_i] = 0
                print("from to", from_sample_i, to_sample_i, " = 0")
        
        # quit()
        for from_set_id in range(n):
            for from_idx_in_ts in range(len(taget_samples[from_set_id])):
            
                for to_set_id in range(n):
                    if from_set_id != to_set_id:
                        
                        for to_idx_in_ts in range(len(taget_samples[to_set_id])):
                            from_node = taget_samples[from_set_id][from_idx_in_ts]
                            to_node = taget_samples[to_set_id][to_idx_in_ts]
                            to_idx_in_ts_plus_one = to_idx_in_ts + 1
                            if to_idx_in_ts_plus_one >= len(taget_samples[to_set_id]):
                                to_node_plus_one = taget_samples[to_set_id][0]
                            else:
                                to_node_plus_one = taget_samples[to_set_id][to_idx_in_ts_plus_one]
                            
                            self.distances[from_node][to_node_plus_one] = all_distances[from_node][to_node] + M
        
        # TODO - deal with start_idx 
        #      - special case for single node cluster....... 

        # sequence = self.compute_tsp_sequence(start_idx=start_idx)
        sequence = self.compute_tsp_sequence()
        
        selected_samples = []
        for i in range(len(sequence)):
            set_prew = sample_target[sequence[i - 1]]
            set_now = sample_target[sequence[i]]
            
            # print(sequence[i - 1], "in", set_prew)
            # print(sequence[i ], "in", set_now)
            if set_prew != set_now:
                # print("add ",sequence[i - 1])
                selected_samples.append(sequence[i - 1])
            # print("------")
        
        if len(selected_samples) > 0 and sample_target[selected_samples[0]] != start_idx:
            for i in range(len(selected_samples)):
                if sample_target[selected_samples[i]] == start_idx:
                    new_sequence = selected_samples[i:len(selected_samples)] + selected_samples[0:i]
                    selected_samples = new_sequence
                    break

        print("k", k)
        print(selected_samples)        

        path = []
        for a in range(0, n):
            b = (a + 1) % n
            start = samples[selected_samples[a]]
            end = samples[selected_samples[b]]
            if turning_radius == 0:
                if a == 0:
                    path.append(start[0:2])
                path.append(end[0:2])
            else:
                if a == 0:
                    path.append(start)
                path.append(end)
        
        print("plan_tour_dtspn_noon_bean path", path)
        return path

        # #} end of plan_tour_dtspn_noon_bean()
        
