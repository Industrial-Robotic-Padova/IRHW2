
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
        self.torso_cmd = rospy.Publisher(
            '/torso_controller/command', JointTrajectory, queue_size=1.0)
        self.arm_cmd = rospy.Publisher(
            '/arm_controller/command', JointTrajectory, queue_size=1.0)
        rospy.loginfo("Moving head down")
        self.play_m_as = SimpleActionClient('/play_motion', PlayMotionAction)
        if not self.play_m_as.wait_for_server(rospy.Duration(20)):
            rospy.logerr("Could not connect to /play_motion AS")
            exit()
        rospy.loginfo("Connected!")
        rospy.sleep(1.0)
        rospy.loginfo("Done initializing PickAruco.")
        self.move_head()
        self.move_torso()
        #self.open_arm()
        pmg = PlayMotionGoal()
        pmg.motion_name = 'pick_final_pose'
        pmg.skip_planning = False
        self.play_m_as.send_goal_and_wait(pmg)
        rospy.loginfo("Raise object done.")

    def move_head(self):
        jt = JointTrajectory()
        jt.joint_names = ['head_1_joint', 'head_2_joint']
        jtp = JointTrajectoryPoint()
        jtp.positions = [0.0, -0.75]
        jtp.time_from_start = rospy.Duration(2.0)
        jt.points.append(jtp)
        self.head_cmd.publish(jt)
        rospy.loginfo("Done.")

    def move_torso(self):
        rospy.loginfo("Moving torso up")
        jt = JointTrajectory()
        jt.joint_names = ['torso_lift_joint']
        jtp = JointTrajectoryPoint()
        jtp.positions = [0.34]
        jtp.time_from_start = rospy.Duration(2.5)
        jt.points.append(jtp)
        self.torso_cmd.publish(jt)

    def open_arm(self):
        jt = JointTrajectory()
        jt.joint_names = ['arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint']
        jtp = JointTrajectoryPoint()
        #jtp.positions = [2.5, 0.2, -2.1, 1.9, 1.0, -0.5, 0.0]
        jtp.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        jtp.time_from_start = rospy.Duration(2.0)
        jt.points.append(jtp)
        self.arm_cmd.publish(jt)
        rospy.loginfo("Done.")


if __name__ == "__main__":
    # Create a node
    rospy.init_node("demo")

    # Setup clients
    head_action = PointHeadClient()
    rospy.spin()

