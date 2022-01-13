#! /usr/bin/env python
from __future__ import print_function

import sys
import rospy
import actionlib

from tiago_iaslab_simulation.msg import IRMoveAction, IRMoveGoal, IRMoveFeedback, IRMoveResult
from tiago_iaslab_simulation.srv import Objs

positions_ = {
    'table': (8, -2, -0.68),
    '1': (10.5, 0.45, -0.9),
    '2': (11.5, 0.45, -0.9),
    '3': (12.5, 0.45, -0.9),
}


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
            send_pose(positions_.get('table'))
            # pick
            send_pose(positions_.get(str(id_)))
            # put

    except rospy.ROSInterruptException:
        print("program interrupted before completion")
