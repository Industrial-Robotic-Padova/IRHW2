#! /usr/bin/env python
from __future__ import print_function

import sys
import rospy
import actionlib

from tiago_iaslab_simulation.msg import IRMoveAction, IRMoveGoal, IRMoveFeedback, IRMoveResult
from tiago_iaslab_simulation.srv import Objs

positions_ = {
    'table': (8, -2, -0.68),
    '1': (10.5, 0.75, -0.9),
    '2': (11.5, 0.75, -0.9),
    '3': (12.5, 0.75, -0.9),
}
coords = {
    'base': (-6.580047, 1.369940),
    'table': (1.245143, -1.613171),
    '1': (4.007396, 1.015966),
    '2': (5.007404, 1.015966),
    '3': (6.007146, 1.015966),
}
table_size = 0.913
cylinder_size = 0.21
robot_size = 0.7


def pose_calc(obj_str):
    return coords.get(obj_str)[0] - coords.get('base')[0], coords.get(obj_str)[1] - coords.get('base')[1]


def pose_calc_cyl(obj_str):
    pose_table = list(pose_calc(obj_str))
    pose_table[1] -= (cylinder_size + robot_size/2)
    pose_table.append(0.68)
    return tuple(pose_table)


def pose_calc_table(angle):
    pose_table = list(pose_calc('table'))
    if angle == 1:
        pose_table[0] += table_size
        pose_table[1] += (table_size + robot_size/2)
        pose_table.append(-0.68)
    return tuple(pose_table)


def send_pose(pos):
    client = actionlib.SimpleActionClient('/ir_pose', IRMoveAction)
    client.wait_for_server()

    goal = IRMoveGoal(position=pos)
    client.send_goal(goal)
    client.wait_for_result()
    return client.get_result()


def get_obj_ids():
    try:
        rospy.wait_for_service('human_objects_srv')
        get_ids = rospy.ServiceProxy('human_objects_srv', Objs)
        res = get_ids(1, 1)
        return res.ids

    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


if __name__ == '__main__':
    try:
        rospy.init_node('ir_pose_client_test')
        result = send_pose([8, 0, 0])
        print("Result:", result.status)
        ids_ = get_obj_ids()
        print("ids:", result)
        for id_ in ids_:
            send_pose(pose_calc_table(angle=1))
            # pick
            send_pose(pose_calc_cyl(str(id_)))
            # put

    except rospy.ROSInterruptException:
        print("program interrupted before completion")
