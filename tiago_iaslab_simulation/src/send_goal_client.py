#! /usr/bin/env python
import rospy
import time
import actionlib
from move_base_msgs import msg


def feedback_callback(feedback):
    print('[Feedback] Going to Goal Pose...')


rospy.init_node('move_base_action_client')

client = actionlib.SimpleActionClient('/move_base', msg.MoveBaseAction)
client.wait_for_server()

while True:
    val = input("Enter your value: ")
    print(val)
    x, y, Rz = val
    # creates a goal to send to the action server
    goal = msg.MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = 0.0
    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = Rz
    goal.target_pose.pose.orientation.w = 0.66

    # sends the goal to the action server
    client.send_goal(goal, feedback_cb=feedback_callback)
    client.wait_for_result()

    print('[Result] State: %d' % (client.get_state()))
