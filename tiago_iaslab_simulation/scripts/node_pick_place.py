#! /usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from actionlib import SimpleActionClient, SimpleActionServer

from tiago_iaslab_simulation import msg as ir_msg

from geometry_msgs import msg as geometry_msgs
from moveit_msgs import msg as moveit_msgs
# from apriltag_ros.msg import AprilTagDetectionArray
import moveit_commander
# from moveit_msgs.msg import CollisionObject
# from moveit_msgs.srv import GetPositionIKRequest, GraspPlanning, GetPositionIK
from math import pi


class PickAndPlaceServer(object):

    def __init__(self):
        self.torso_cmd = rospy.Publisher('/torso_controller/command', JointTrajectory, queue_size=1)
        self.head_cmd = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=1)

        # Get the object size
        self.object_height = rospy.get_param('~object_height')
        self.object_width = rospy.get_param('~object_width')
        self.object_depth = rospy.get_param('~object_depth')

        self.pick_as = SimpleActionServer('/pickup_pose', ir_msg.IRPickPlaceAction, execute_cb=self.pick_cb, auto_start=False)
        self.pick_as.start()

    def lift_torso(self):
        # Move torso to its maximum height
        rospy.loginfo("Moving torso up")
        jt = JointTrajectory()
        jt.joint_names = ['torso_lift_joint']
        jtp = JointTrajectoryPoint()
        jtp.positions = [0.34]
        jtp.time_from_start = rospy.Duration(2.5)
        jt.points.append(jtp)
        self.torso_cmd.publish(jt)

    def lower_head(self):
        rospy.loginfo("Moving head down")
        jt = JointTrajectory()
        jt.joint_names = ['head_1_joint', 'head_2_joint']
        jtp = JointTrajectoryPoint()
        jtp.positions = [0.0, -0.75]
        jtp.time_from_start = rospy.Duration(2.0)
        jt.points.append(jtp)
        self.head_cmd.publish(jt)
        rospy.loginfo("Done.")

    def move_arm_safe(self):
        rospy.loginfo("Moving arm to a safe pose")
        moveit_commander.roscpp_initialize(sys.argv)
        robot = moveit_commander.RobotCommander()
        move_group = moveit_commander.MoveGroupCommander("arm_torso")

        print("============ Planning frame: %s" % move_group.get_planning_frame())
        print("============ End effector link: %s" % move_group.get_end_effector_link())
        print("============ Available Planning Groups:", robot.get_group_names())
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")

        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = 1.57
        joint_goal[2] = -1
        joint_goal[3] = 0
        joint_goal[4] = 1
        joint_goal[5] = 0
        joint_goal[6] = 0
        move_group.go(joint_goal, wait=True)
        move_group.stop()

    def pick_cb(self, goal):
        """
        :type goal: PickUpPoseGoal
        """
        self.lift_torso()
        self.lower_head()
        self.move_arm_safe()
        self.move_arm(goal.object_pose)
        p_res = ir_msg.IRPickPlaceResult()
        # error_code = self.grasp_object(goal.object_pose)
        error_code = 1
        p_res.error_code = error_code
        if error_code != 1:
            self.pick_as.set_aborted(p_res)
        else:
            self.pick_as.set_succeeded(p_res)

    # def grasp_object(self, object_pose):
    #     rospy.loginfo("Object pose: %s", object_pose.pose)
    #
    #     table_pose = copy.deepcopy(object_pose)
    #
    #     # define a virtual table below the object
    #     table_height = object_pose.pose.position.z - self.object_width / 2
    #     table_width = 1.8
    #     table_depth = 0.5
    #     table_pose.pose.position.z += -(2 * self.object_width) / 2 - table_height / 2
    #     table_height -= 0.008  # remove few milimeters to prevent contact between the object and the table
    #
    #     # compute grasps
    #     possible_grasps = self.sg.create_grasps_from_object_pose(object_pose)
    #     self.pickup_ac
    #     goal = createPickupGoal("arm_torso", "part", object_pose, possible_grasps, self.links_to_allow_contact)
    #
    #     rospy.loginfo("Sending goal")
    #     self.pickup_ac.send_goal(goal)
    #     rospy.loginfo("Waiting for result")
    #     self.pickup_ac.wait_for_result()
    #     result = self.pickup_ac.get_result()
    #     rospy.logdebug("Using torso result: " + str(result))
    #     rospy.loginfo("Pick result: " + str(moveit_error_dict[result.error_code.val]))
    #
    #     return result.error_code.val

    def move_arm(self, pose_stamped):
        self.group = None
        self.object_pose = geometry_msgs.PoseStamped()

        moveit_commander.roscpp_initialize(sys.argv)
        robot = moveit_commander.RobotCommander()
        move_group = moveit_commander.MoveGroupCommander("arm_torso")

        print("============ Planning frame: %s" % move_group.get_planning_frame())
        print("============ End effector link: %s" % move_group.get_end_effector_link())
        print("============ Available Planning Groups:", robot.get_group_names())
        print("============ Printing robot state", robot.get_current_state())
        print()

        pose_goal = pose_stamped.pose
        # pose_goal = geometry_msgs.Pose()
        # pose_goal.position.x = 0.4
        # pose_goal.position.y = -0.3
        # pose_goal.position.z = 0.26
        # pose_goal.orientation.x = -0.011
        # pose_goal.orientation.y = 1.57
        # pose_goal.orientation.z = 0.037
        # pose_goal.orientation.w = 1.0

        move_group.set_pose_target(pose_goal)
        plan = move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()


if __name__ == '__main__':
    rospy.init_node('ir_pick_place')
    sphere = PickAndPlaceServer()
    rospy.spin()
