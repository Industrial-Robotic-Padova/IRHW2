#! /usr/bin/env python
import rospy
import time
import actionlib
from move_base_msgs import msg as msg_move_base
from trajectory_msgs import msg as msg_trajectory
from moveit_msgs import msg as msg_moveit
from geometry_msgs import msg as msg_geometry


def move_head():
    rospy.loginfo("Moving head down")
    jt = msg_trajectory.JointTrajectory()
    jt.joint_names = ['head_1_joint', 'head_2_joint']
    jtp = msg_trajectory.JointTrajectoryPoint()
    jtp.positions = [0.0, -0.75]
    jtp.time_from_start = rospy.Duration(2.0)
    jt.points.append(jtp)
    self.head_cmd.publish(jt)
    rospy.loginfo("Done.")


def get_object_number():
    pass


def get_image_to_aprilTag():
    pass


class HW2:
    def __init__(self):
        rospy.init_node('node_main')

        rospy.loginfo("Setting publishers to torso and head controller...")
        self.head_cmd = rospy.Publisher('/head_controller/command', msg_trajectory.JointTrajectory, queue_size=1)
        rospy.loginfo("Waiting for '/play_motion' AS...")

        self.play_m_as = actionlib.SimpleActionClient('/play_motion', msg_moveit.PlayMotionAction)
        if not self.play_m_as.wait_for_server(rospy.Duration(20)):
            rospy.logerr("Could not connect to /play_motion AS")
            exit()
        rospy.loginfo("Connected!")

    def move_to_table(self, pose_table):
        def feedback_callback(feedback):
            print('[Feedback] Going to Goal Pose...')

        client = actionlib.SimpleActionClient('/move_base', msg_move_base.MoveBaseAction)
        client.wait_for_server()

        x, y, Rz = pose_table
        # creates a goal to send to the action server
        goal = msg_move_base.MoveBaseGoal()
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

    def pick(self, object_pose, possible_grasps):
        rospy.loginfo("Connecting to pickup AS")
        self.pickup_ac = actionlib.SimpleActionClient('/pickup', msg_moveit.PickupAction)
        self.pickup_ac.wait_for_server()
        rospy.loginfo("Succesfully connected.")

        def createPickupGoal(group="arm_torso", target="part", grasp_pose=msg_geometry.PoseStamped(), possible_grasps=[], links_to_allow_contact=None):
            """ Create a PickupGoal with the provided data"""
            pug = msg_moveit.PickupGoal()
            pug.target_name = target
            pug.group_name = group
            pug.possible_grasps.extend(possible_grasps)
            pug.allowed_planning_time = 35.0
            pug.planning_options.planning_scene_diff.is_diff = True
            pug.planning_options.planning_scene_diff.robot_state.is_diff = True
            pug.planning_options.plan_only = False
            pug.planning_options.replan = True
            pug.planning_options.replan_attempts = 1  # 10
            pug.allowed_touch_objects = []
            pug.attached_object_touch_links = ['<octomap>']
            pug.attached_object_touch_links.extend(links_to_allow_contact)

            return pug

        links_to_allow_contact = ["gripper_left_finger_link", "gripper_right_finger_link", "gripper_link"]

        goal = createPickupGoal("arm_torso", "part", object_pose, possible_grasps, links_to_allow_contact)

        rospy.loginfo("Sending goal")
        self.pickup_ac.send_goal(goal)
        rospy.loginfo("Waiting for result")
        self.pickup_ac.wait_for_result()
        result = self.pickup_ac.get_result()
        rospy.logdebug("Using torso result: " + str(result))


if __name__ == "__main__":
    hw2 = HW2()
    pose_table = ()
    hw2.move_to_table(pose_table)
