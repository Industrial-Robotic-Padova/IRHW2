#! /usr/bin/env python
import rospy
import actionlib

from tiago_iaslab_simulation.msg import IRMoveAction, IRMoveGoal, IRMoveFeedback, IRMoveResult


def ir_pose_client(pos):
    client = actionlib.SimpleActionClient('/ir_pose', IRMoveAction)
    client.wait_for_server()

    goal = IRMoveGoal(position=pos)
    client.send_goal(goal)
    client.wait_for_result()
    return client.get_result()


if __name__ == '__main__':
    try:
        rospy.init_node('ir_pose_client_test')
        result = ir_pose_client([8, 0, 0])
        print("Result:", result.status)
    except rospy.ROSInterruptException:
        print("program interrupted before completion")
