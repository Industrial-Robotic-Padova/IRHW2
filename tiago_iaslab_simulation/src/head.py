#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

rospy.init_node('troso')

torso_cmd = rospy.Publisher('/torso_controller/command', JointTrajectory, queue_size=1)
rospy.loginfo("Moving torso up")
print(torso_cmd)
jt = JointTrajectory()
jt.joint_names = ['torso_lift_joint']
jtp = JointTrajectoryPoint()
jtp.positions = [0.34]
jtp.time_from_start = rospy.Duration(2.5)
jt.points.append(jtp)
torso_cmd.publish(jt)
rospy.loginfo("done")
