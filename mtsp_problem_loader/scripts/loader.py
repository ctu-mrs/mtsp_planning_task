#!/usr/bin/env python2

import rospy

# MRS ROS messages
from std_srvs.srv import Trigger
from std_srvs.srv import TriggerResponse
from mtsp_msgs.msg import TspProblem
from mtsp_msgs.msg import TspPoint
from std_msgs.msg import Int32

# TSP
from mtsp_problem_loader.tsp_problem import *

class TspLoader:

    def callbackStart(self, req):

        response = TriggerResponse();
        response.success = True
        response.message = "loading the problem"

        # load the problem
        problem = MTSPProblem.load_problem(self.filename)

        rospy.loginfo('TSP problem loaded')

        # create the ROS message
        tsp_problem_msg = TspProblem()
        tsp_problem_msg.name = problem.name
        tsp_problem_msg.type = problem.type
        tsp_problem_msg.comment = problem.comment
        tsp_problem_msg.dimension = problem.dimension
        tsp_problem_msg.number_of_robots = problem.number_of_robots
        
        tsp_problem_msg.neighborhood_radius = problem.neighborhood_radius
        tsp_problem_msg.edge_weight_type = problem.edge_weight_type
        tsp_problem_msg.height = self.height

        for idx,point in enumerate( problem.start_positions):
            
            # start point
            start_point_msg = TspPoint()

            start_point_msg.idx = point[0]
            start_point_msg.x = point[1]
            start_point_msg.y = point[2]
            
            tsp_problem_msg.start_points.append(start_point_msg)

        # fill in the individual points
        for idx,point in enumerate(problem.targets):

            # TSP point
            tsp_point_msg = TspPoint()

            tsp_point_msg.idx = point[0]
            tsp_point_msg.x = point[1]
            tsp_point_msg.y = point[2]

            tsp_problem_msg.points.append(tsp_point_msg)

        # publish the ros message

        self.publisher_problem.publish(tsp_problem_msg)

        rospy.loginfo('Problem published, staying on, the publisher is latched.')

        return response

    def __init__(self):

        rospy.init_node('tsp_loader', anonymous=True)

        # load parameters
        self.filename = rospy.get_param('~filename', '')
        self.height = rospy.get_param('~height', '')

        # initiate ROS publishers
        self.publisher_problem = rospy.Publisher("~tsp_problem_out", TspProblem, queue_size=1, latch=True)

        # initiate ROS service servers
        self.service_server_start = rospy.Service('~start_in', Trigger, self.callbackStart)

        rospy.spin()

if __name__ == '__main__':
    try:
        tsp_loader = TspLoader()
    except rospy.ROSInterruptException:
        pass
