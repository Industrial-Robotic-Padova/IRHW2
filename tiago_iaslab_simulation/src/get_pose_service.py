#! /usr/bin/env python

import rospy
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
from sensor_msgs.msg import LaserScan
import numpy as np
import math
from utils_ import *


robot_pos = np.array([])
obs = []

robot_pose = Pose()


def service_callback(request):
    print("Robot Pose:", robot_pose)
    return EmptyResponse()


def sub_callback(msg):
    print(msg.pose.pose)
    global robot_pos
    robot_pos = get_trans_matrix_robot(
        msg.pose.pose.orientation.z,
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        msg.pose.pose.position.z
    )


def laser_callback(msg):
    ranges_ = list(msg.ranges)

    for obstacle_index_range in detect_obstacles(ranges_):
        obstacle_index_center = int((obstacle_index_range[0] + obstacle_index_range[1]) / 2)
        d, r = get_obstacle_distance(
            obstacle_index_range[0],
            obstacle_index_range[1],
            ranges_[obstacle_index_range[0]],
            ranges_[obstacle_index_center]
        )

        x_, y_ = get_obstacle_position(
            d,
            obstacle_index_center,
            robot_pos[0][3],
            robot_pos[1][3],
        )
        if not is_obstacle_exists((x_, y_), obs):
            obs.append((x_, y_))

    if len(obs) > 0:
        print('FINAL POSES: ', obs)


rospy.init_node('service_server')
# create the Service called get_pose_service with the defined callback
my_service = rospy.Service('/get_pose_service', Empty, service_callback)
sub_pose = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, sub_callback)
laser_msg = rospy.Subscriber('/scan', LaserScan, laser_callback)
# maintain the service open.
rospy.spin()
