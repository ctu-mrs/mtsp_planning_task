"""
Custom TSP Loader
@author: R.Penicka
"""

import rospy

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import math
import time
from mrs_msgs.msg import Reference
from mrs_msgs.msg import TrajectoryReference
import dubins

# #{ dist_euclidean_squared()

def dist_euclidean_squared(coord1, coord2):
    """ euclidean distance between coord1 and coord2"""
    (x1, y1) = coord1
    (x2, y2) = coord2
    return (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)

# #} end of dist_euclidean_squared()

# #{ dist_euclidean()

def dist_euclidean(coord1, coord2):
    """ euclidean distance between coord1 and coord2"""
    return math.sqrt(dist_euclidean_squared(coord1, coord2))

# #} end of dist_euclidean()

# #{ pos_in_distance()

def pos_in_distance(start , stop , dist):
    dist_tot = dist_euclidean(start, stop)
    (x1, y1) = start
    (x2, y2) = stop

    x = x1 + (x2 - x1) * dist / dist_tot
    y = y1 + (y2 - y1) * dist / dist_tot

    return [x, y]

# #} end of pos_in_distance()

class TSPTrajectory():

    # #{ __init__()

    def __init__(self, max_velocity, max_acceleration):
        self.time_sample = 0.2
        self.max_velocity = max_velocity
        self.max_acceleration = max_acceleration

    # #} end of __init__()

    # #{ sample_trajectory_euclidean()

    def sample_euclidean_with_stops(self, start, stop, init_velocity=0, final_velocity=0, sample_start_time=0):
        """ euclidean trajectory with stops between start and stop"""

        #print("sample_euclidean_with_stops from", start, "to", stop)

        samples = []
        trajectory_part_time = 0
        # acc = 0 # no jeck jet
        dist_total = dist_euclidean(start, stop)
        #print("dist_total", dist_total)

        time_from_init_to_max_vel = (self.max_velocity - init_velocity) / self.max_acceleration
        time_from_max_to_final_vel = (self.max_velocity - final_velocity) / self.max_acceleration

        dist_from_init_to_max_vel = 0.5 * (self.max_velocity + init_velocity) * time_from_init_to_max_vel  # average speed * time
        dist_from_max_vel_to_final = 0.5 * (self.max_velocity + final_velocity) * time_from_max_to_final_vel  # average speed * time

        """
        print("time_from_init_to_max_vel", time_from_init_to_max_vel, "s")
        print("time_from_max_to_final_vel", time_from_max_to_final_vel, "s")
        print("dist_from_init_to_max_vel", dist_from_init_to_max_vel, "m")
        print("dist_from_max_vel_to_final", dist_from_max_vel_to_final, "m")
        """

        if dist_total < dist_from_init_to_max_vel + dist_from_max_vel_to_final: # can not reach maximal speed in straigh line
            #print("can not reach max vel in trajectory")
            t = 0
            sample = 0

            if init_velocity == 0 and final_velocity == 0:
                time_to_possible_max_vel = math.sqrt(dist_total / self.max_acceleration)
                velocity_in_middle = time_to_possible_max_vel * self.max_acceleration
                trajectory_part_time = 2 * time_to_possible_max_vel
            else:
                if init_velocity > final_velocity:  # initial velocity is larger than final, in the end is additinal decelerating
                    time_final_decel = (init_velocity - final_velocity) / self.max_acceleration
                    dist_final_decel = time_final_decel * (init_velocity + final_velocity) * 0.5
                    dist_acc_decc = dist_total - dist_final_decel
                    time_to_possible_max_vel = (-init_velocity + math.sqrt(init_velocity ** 2 + self.max_acceleration * dist_acc_decc)) / self.max_acceleration
                    velocity_in_middle = init_velocity + time_to_possible_max_vel * self.max_acceleration
                    trajectory_part_time = time_to_possible_max_vel + time_final_decel
                else:
                    time_init_accel = (final_velocity - init_velocity) / self.max_acceleration
                    dist_init_accel = time_init_accel * (init_velocity + final_velocity) * 0.5
                    dist_acc_decc = dist_total - dist_init_accel
                    time_to_possible_max_vel = time_init_accel + (-final_velocity + math.sqrt(final_velocity ** 2 + self.max_acceleration * dist_acc_decc)) / self.max_acceleration
                    velocity_in_middle = init_velocity + time_to_possible_max_vel * self.max_acceleration

                    """
                    print("time_init_accel", time_init_accel)
                    print("dist_init_accel", dist_init_accel)
                    print("dist_total", dist_total)
                    print("dist_acc_decc", dist_acc_decc)
                    print("such dist is", 0.5 * (velocity_in_middle + init_velocity) * time_to_possible_max_vel * 2)
                    """
                    trajectory_part_time = 2 * time_to_possible_max_vel - time_init_accel

            """
            print("time_to_possible_max_vel", time_to_possible_max_vel)
            print("velocity_in_middle", velocity_in_middle)
            print("sample_start_time", sample_start_time)
            """


            while (sample + 1) * self.time_sample <= time_to_possible_max_vel - sample_start_time:
                t = (sample + 1) * self.time_sample + sample_start_time
                sample += 1
                v = init_velocity + self.max_acceleration * t
                s = init_velocity * t + 0.5 * self.max_acceleration * (t ** 2)
                #print("t", t, "v", v, "s", s, "sample", sample)
                pos_in_dist = pos_in_distance(start, stop, s)
                samples.append(pos_in_dist)


            #print("end acc")

            while (sample + 1) * self.time_sample <= trajectory_part_time - sample_start_time:
                t = (sample + 1) * self.time_sample + sample_start_time
                sample += 1
                t_part = t - time_to_possible_max_vel
                v = velocity_in_middle - self.max_acceleration * t_part
                s = time_to_possible_max_vel * 0.5 * (velocity_in_middle + init_velocity) + velocity_in_middle * t_part - 0.5 * self.max_acceleration * (t_part ** 2)
                #print("t", t, "v", v, "s", s, "sample", sample)
                pos_in_dist = pos_in_distance(start, stop, s)
                samples.append(pos_in_dist)

            #print("end decc")

        else:  # can reach maximal speed in straigh line
            #print("can reach max vel")
            dist_constant_speed = dist_total - dist_from_init_to_max_vel - dist_from_max_vel_to_final
            time_constant_speed = dist_constant_speed / self.max_velocity
            trajectory_part_time = time_from_init_to_max_vel + time_constant_speed + time_from_max_to_final_vel
            """
            print("time_constant_speed", time_constant_speed, "s")
            print("dist_constant_speed", dist_constant_speed, "m")
            print("trajectory_part_time", trajectory_part_time)
            """
            t = 0
            sample = 0
            while (sample + 1) * self.time_sample <= time_from_init_to_max_vel - sample_start_time:
                t = (sample + 1) * self.time_sample + sample_start_time
                sample += 1
                v = init_velocity + self.max_acceleration * t
                s = init_velocity * t + 0.5 * self.max_acceleration * (t ** 2)
                pos_in_dist = pos_in_distance(start, stop, s)
                samples.append(pos_in_dist)
                #print("t", t, "v", v, "s", s, "sample", sample)

            #print("end acc")

            while (sample + 1) * self.time_sample <= time_from_init_to_max_vel + time_constant_speed - sample_start_time:
                t = (sample + 1) * self.time_sample + sample_start_time
                sample += 1
                t_part = t - time_from_init_to_max_vel
                v = self.max_velocity
                s = dist_from_init_to_max_vel + v * t_part
                pos_in_dist = pos_in_distance(start, stop, s)
                samples.append(pos_in_dist)
                #print("t", t, "v", v, "s", s, "sample", sample)

            #print("end const")

            while (sample + 1) * self.time_sample <= time_from_init_to_max_vel + time_constant_speed + time_from_max_to_final_vel - sample_start_time:
                t = (sample + 1) * self.time_sample + sample_start_time
                sample += 1
                t_part = t - (time_from_init_to_max_vel + time_constant_speed)
                v = self.max_velocity - self.max_acceleration * t_part
                s = (dist_total - dist_from_max_vel_to_final) + self.max_velocity * t_part - 0.5 * self.max_acceleration * (t_part ** 2)
                #print("t", t, "v", v, "s", s, "sample", sample)
                pos_in_dist = pos_in_distance(start, stop, s)
                samples.append(pos_in_dist)

            if final_velocity == 0 and samples[-1] != stop:
                #print("t last", "v", 0, "s", dist_total)
                samples.append(stop)

        return samples, trajectory_part_time

    # #} end of sample_trajectory_euclidean()

    # #{ sample_trajectory_euclidean()

    def sample_trajectory_euclidean(self, sequence):

        """ sample euclidean tarjectory over sequence """

        print("sample_trajectory_euclidean in sequence", sequence)

        samples = []
        samples.append(sequence[0])  # add first point of trajectory
        trajectory_time = 0

        for target_id in range(1, len(sequence)):
            from_target = sequence[target_id - 1]
            to_target = sequence[target_id]
            part_samples, part_time = self.sample_euclidean_with_stops(from_target, to_target)
            trajectory_time += part_time
            #print("part_time", part_time)
            samples.extend(part_samples)
            # return samples, trajectory_time

        return samples, trajectory_time

    # #} end of sample_trajectory_euclidean()

    # #{ sample_trajectory_dubins()

    def sample_trajectory_dubins(self, sequence, turning_velocity=None):
        """ sample dubins tarjectory over sequence """

        print("sample_trajectory_dubins in sequence", sequence)

        if turning_velocity is None:
            turning_velocity = self.max_velocity
        print("using turning_velocity", turning_velocity ," and acceleration ",self.max_acceleration)


        turning_radius = (turning_velocity * turning_velocity) / self.max_acceleration
        print("which means turning_radius", turning_radius)

        sequence_start = 0
        init_velocity = 0
        time_to_turning_velocity = (turning_velocity - init_velocity) / self.max_acceleration
        dist_to_turning_velocity = 0.5 * (turning_velocity + init_velocity) * time_to_turning_velocity  # average speed * time

        samples = []
        sample = 0
        t = 0
        last_segment_end_time = 0
        next_sample_start_time = 0
        samples.append(sequence[sequence_start])
        #print("t", t, "v", 0, "s", 0, "sample", sample, "start")

        for target_id in range(sequence_start + 1, len(sequence)):
            start = sequence[target_id - 1]
            end = sequence[target_id]
            dubins_path = dubins.shortest_path(start, end, turning_radius)

            """
            print("segment 0", dubins_path.segment_length(0))
            print("segment 1", dubins_path.segment_length(1))
            print("segment 2", dubins_path.segment_length(2))
            print("dubins lenght", dubins_path.path_length())
            """

            # first segment of dubins
            if (sample + 1) * self.time_sample < time_to_turning_velocity:
                init_velocity = last_segment_end_time * self.max_acceleration
                time_accel = (turning_velocity - init_velocity) / self.max_acceleration
                dist_accel = init_velocity * time_accel + 0.5 * self.max_acceleration * time_accel * time_accel

                if dubins_path.segment_length(0) < dist_accel :  # accel whole time
                    segment_1_time = (-init_velocity + math.sqrt(init_velocity ** 2 + 2 * self.max_acceleration * dubins_path.segment_length(0))) / self.max_acceleration  # turning segment 0
                else:  # accel only part time
                    segment_1_time = time_accel + (dubins_path.segment_length(0) - dist_accel) / turning_velocity

            else:
                segment_1_time = dubins_path.segment_length(0) / turning_velocity  # turning segment 0
                init_velocity = turning_velocity


            #print("last_segment_end_time", last_segment_end_time)
            #print("segment_1_time",segment_1_time)

            acc_time = turning_velocity/self.max_acceleration
            segment_1_time_dist = 0.5*self.max_acceleration*acc_time*acc_time + (segment_1_time-acc_time)*turning_velocity


            while (sample + 1) * self.time_sample <= last_segment_end_time + segment_1_time:
                t = (sample + 1) * self.time_sample - last_segment_end_time

                if init_velocity != turning_velocity:
                    if (sample + 1) * self.time_sample <= time_to_turning_velocity: # still accelerating from init_velocity
                        s = init_velocity * t + 0.5 * self.max_acceleration * (t ** 2)
                    else:
                        dist_init_acc = 0.5 * (turning_velocity + init_velocity) * time_to_turning_velocity  # alreaddy accelerated from init_velocity to turning_velocity

                        time_after_init_acc = t - time_to_turning_velocity
                        s = dist_init_acc + turning_velocity * time_after_init_acc

                else: # already turning velocity from begining
                    s = turning_velocity * t

                sample += 1
                samples.append(dubins_path.sample(s))
                #print("t", t, "s", s, "sample", sample,"sample time", sample*self.time_sample, "dubins length", dubins_path.path_length(), "dubins part len", dubins_path.segment_length(0), "rot1")

            last_segment_end_time += segment_1_time
            next_sample_start_time = sample * self.time_sample - last_segment_end_time

            if last_segment_end_time < time_to_turning_velocity:
                init_velocity = last_segment_end_time * self.max_acceleration
            else:
                init_velocity = turning_velocity

            #print("---------- end fist segment --------------- at time", last_segment_end_time)


            # second segment of Dubins
            if dubins_path.path_type() != dubins.LRL and dubins_path.path_type() != dubins.RLR: # straight line segment
                start_straight_line = dubins_path.sample(dubins_path.segment_length(0))
                stop_straight_line = dubins_path.sample(dubins_path.segment_length(0) + dubins_path.segment_length(1))

                """
                print("start_straight_line", start_straight_line)
                print("stop_straight_line", stop_straight_line)
                print("init_velocity", init_velocity)
                print("final_velocity", turning_velocity)
                print("next_sample_start_time", next_sample_start_time)
                """

                straight_line_samples, segment_2_time = self.sample_euclidean_with_stops(start_straight_line[0:2], stop_straight_line[0:2], init_velocity=init_velocity, final_velocity=turning_velocity, sample_start_time=next_sample_start_time)
                phi = start_straight_line[2]
                straight_line_samples_w_head = [(x, y, phi) for x, y in straight_line_samples]
                samples += straight_line_samples_w_head
                sample += len(straight_line_samples_w_head)
            else: # also circular segment
                segment_2_time = dubins_path.segment_length(1) / turning_velocity  # turning segment 1
                while (sample + 1) * self.time_sample <= last_segment_end_time + segment_2_time:
                    t = (sample + 1) * self.time_sample - last_segment_end_time
                    sample += 1
                    # t_part = t - last_segment_end_time
                    s = dubins_path.segment_length(0) + turning_velocity * t
                    samples.append(dubins_path.sample(s))
                    #print("t", t, "s", s, "sample", sample, "dubins length", dubins_path.path_length(), "rot middle")

            last_segment_end_time += segment_2_time
            next_sample_start_time = sample * self.time_sample - last_segment_end_time
            if (sample + 1) * self.time_sample < time_to_turning_velocity:
                init_velocity = last_segment_end_time * self.max_acceleration
            else:
                init_velocity = turning_velocity


            #print("---------- end second segment --------------- at time", last_segment_end_time)

            segment_3_time = dubins_path.segment_length(2) / turning_velocity  # turning segment 2
            while (sample + 1) * self.time_sample <= last_segment_end_time + segment_3_time:
                t = (sample + 1) * self.time_sample - last_segment_end_time
                sample += 1
                if init_velocity != turning_velocity:
                    s = dubins_path.segment_length(0) + dubins_path.segment_length(1) + init_velocity * t + 0.5 * self.max_acceleration * (t ** 2)
                else:
                    s = dubins_path.segment_length(0) + dubins_path.segment_length(1) + turning_velocity * t

                samples.append(dubins_path.sample(s))
                #print("t", t, "v", turning_velocity, "s", s, "sample", sample, "rot2")

            last_segment_end_time += segment_3_time
            next_sample_start_time = sample * self.time_sample - last_segment_end_time

            #print("---------- end last segment --------------- at time", last_segment_end_time)


        return samples, last_segment_end_time

    # #} end of sample_trajectory_dubins()

    # #{ plot_velocity_profile()

    def plot_velocity_profile(self, samples, color='k',title='Velocity profile'):

        figsize = (8, 5)
        fig = plt.figure(figsize=figsize)
        ax = fig.gca()
        ax.set_title(title)
        ax.set_ylabel('velocity [m/s]')

        ax.set_xlabel('time [s]')

        velocities = [0]
        for i in range(1, len(samples)):
            dist = dist_euclidean(samples[i - 1][0:2], samples[i][0:2])
            velocities.append(dist / self.time_sample)
        velocities_time = [self.time_sample * i for i in range(len(velocities))]

        accelerations = [0]
        for i in range(1, len(velocities)):
            vel_change = velocities[i] - velocities[i - 1]
            accelerations.append(vel_change / self.time_sample)

        accelerations_time = [self.time_sample * i for i in range(len(accelerations))]

        plt.axhline(self.max_velocity, 0, len(velocities), ls='-', color='k')
        plt.plot(velocities_time, velocities, '-', color=color, label='velocity')
        plt.axhline(self.max_acceleration, 0, len(accelerations), ls='-.', color='k')
        plt.axhline(-self.max_acceleration, 0, len(accelerations), ls='-.', color='k')
        plt.plot(accelerations_time, accelerations, '-.', color=color, label='acc')
        ax.legend(loc='upper right')
        ax2 = ax.twinx()
        ax2.set_ylabel('acceleration [m/s^2]')

    # #} end of plot_velocity_profile()

    # #{ create_ros_trajectory()

    def create_ros_trajectory(self, trajectory, height):
        """ create the ROS trajectory object """

        default_heading = 1.57

        trajectory_msg = TrajectoryReference()
        trajectory_msg.fly_now = False
        trajectory_msg.use_heading = True
        trajectory_msg.loop = False
        trajectory_msg.header.frame_id = "gps_origin"
        trajectory_msg.header.stamp = rospy.Time.now();

        for point in trajectory:

            if len(point) == 2:  # x and y

                new_point = Reference()
                new_point.position.x = point[0]
                new_point.position.y = point[1]
                new_point.position.z = height
                new_point.heading = default_heading
                trajectory_msg.points.append(new_point)

            elif len(point) == 3:  # x, y and heading

                new_point = Reference()
                new_point.position.x = point[0]
                new_point.position.y = point[1]
                new_point.position.z = height
                new_point.heading = point[2]
                trajectory_msg.points.append(new_point)

        return trajectory_msg

    # #} end of create_ros_trajectory()
