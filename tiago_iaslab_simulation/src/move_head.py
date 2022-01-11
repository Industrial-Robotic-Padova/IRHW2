#!/usr/bin/env python
import rospy

from control_msgs.msg import PointHeadAction, PointHeadGoal
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from actionlib import SimpleActionClient
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from control_msgs.msg import FollowJointTrajectoryAction
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# Point the head using controller

class PointHeadClient(object):
    def __init__(self):
        rospy.loginfo("Waiting for '/play_motion' AS...")
        self.head_cmd = rospy.Publisher(
            '/head_controller/command', JointTrajectory, queue_size=1.0)
        rospy.loginfo("Moving head down")
        self.play_m_as = SimpleActionClient('/play_motion', PlayMotionAction)
        if not self.play_m_as.wait_for_server(rospy.Duration(20)):
            rospy.logerr("Could not connect to /play_motion AS")
            exit()
        rospy.loginfo("Connected!")
        rospy.sleep(1.0)
        rospy.loginfo("Done initializing PickAruco.")
        self.look_at(2.0, 0.0, 0.6, "base_link")
        pmg = PlayMotionGoal()
        pmg.motion_name = 'pick_final_pose'
        pmg.skip_planning = False
        self.play_m_as.send_goal_and_wait(pmg)
        rospy.loginfo("Raise object done.")

    def look_at(self, x, y, z, frame, duration=1.0):
        jt = JointTrajectory()
        jt.joint_names = ['head_1_joint', 'head_2_joint']
        jtp = JointTrajectoryPoint()
        jtp.positions = [x, z]
        jtp.time_from_start = rospy.Duration(2.0)
        jt.points.append(jtp)
        self.head_cmd.publish(jt)
        rospy.loginfo("Done.")


if __name__ == "__main__":
    # Create a node
    rospy.init_node("demo")

    # Setup clients
    head_action = PointHeadClient()
    rospy.spin()

