#! /usr/bin/env python
import rospy
import time
import actionlib
from move_base_msgs import msg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


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
    head_cmd = rospy.Publisher(
            '/head_controller/command', JointTrajectory, queue_size=1.0)
    jt = JointTrajectory()
    print("head low")
    jt.joint_names = ['head_1_joint', 'head_2_joint']
    jtp = JointTrajectoryPoint()
    jtp.positions = [0, -0.75]
    jtp.time_from_start = rospy.Duration(2.0)
    jt.points.append(jtp)
    head_cmd.publish(jt)
    rospy.loginfo("Done.")
    print('[Result] State: %d' % (client.get_state()))
    torso_cmd = rospy.Publisher(
            '/torso_controller/command', JointTrajectory, queue_size=1)
    rospy.loginfo("Moving torso up")
    jt = JointTrajectory()
    jt.joint_names = ['torso_lift_joint']
    jtp = JointTrajectoryPoint()
    jtp.positions = [0.34]
    jtp.time_from_start = rospy.Duration(2.5)
    jt.points.append(jtp)
    torso_cmd.publish(jt)
