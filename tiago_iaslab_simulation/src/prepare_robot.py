#!/usr/bin/env python
import rospy
from actionlib import SimpleActionClient, SimpleActionServer
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal


def lift_torso():
    torso_cmd = rospy.Publisher(
        '/torso_controller/command', JointTrajectory, queue_size=1)
    rospy.loginfo("Moving torso up")
    jt = JointTrajectory()
    jt.joint_names = ['torso_lift_joint']
    jtp = JointTrajectoryPoint()
    jtp.positions = [-0.34]
    jtp.time_from_start = rospy.Duration(2.5)
    jt.points.append(jtp)
    torso_cmd.publish(jt)


def lower_head():
    head_cmd = rospy.Publisher(
        '/head_controller/command', JointTrajectory, queue_size=1)
    rospy.loginfo("Moving head down")

    jt = JointTrajectory()
    jt.joint_names = ['head_1_joint', 'head_2_joint']
    jtp = JointTrajectoryPoint()
    jtp.positions = [0.0, 0.5]
    jtp.time_from_start = rospy.Duration(3.0)
    jt.points.append(jtp)
    head_cmd.publish(jt)
    rospy.loginfo("Done.")

def open_arm():
    arm_cmd = rospy.Publisher(
        '/arm_controller/command', JointTrajectory, queue_size=10)
    rospy.loginfo("Moving arm")
    jt = JointTrajectory()
    jt.joint_names = ["arm_1_joint",
                      "arm_2_joint",
                      "arm_3_joint",
                      "arm_4_joint",
                      "arm_5_joint",
                      "arm_6_joint",
                      "arm_7_joint"]
    jtp = JointTrajectoryPoint()
    jtp.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    jtp.time_from_start = rospy.Duration(5.0)
    jt.points.append(jtp)
    arm_cmd.publish(jt)
    rospy.loginfo("Done.")


if __name__=='__main__':
    rospy.init_node('prepare_robot', anonymous=True)
    lift_torso()
    lower_head()
    open_arm()
