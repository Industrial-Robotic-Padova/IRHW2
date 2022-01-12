#! /usr/bin/env python
from __future__ import print_function

import sys
import rospy
import actionlib

from tiago_iaslab_simulation.msg import IRMoveAction, IRMoveGoal, IRMoveFeedback, IRMoveResult
from tiago_iaslab_simulation.srv import Objs


def ir_pose_client(pos):
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
        result = ir_pose_client([8, 0, 0])
        print("Result:", result.status)
        result = get_obj_ids()
        print(result)

    except rospy.ROSInterruptException:
        print("program interrupted before completion")
