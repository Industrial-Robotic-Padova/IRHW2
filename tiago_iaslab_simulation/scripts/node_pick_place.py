#! /usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import copy
import rospy
import rospkg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from actionlib import SimpleActionClient, SimpleActionServer

from geometry_msgs import msg as geometry_msgs
from moveit_msgs import msg as moveit_msgs
import moveit_commander

from tiago_iaslab_simulation import msg as ir_msg
from utils import *


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

    def pick_cb(self, goal):
        """
        :type goal: PickUpPoseGoal
        """
        self.lift_torso()
        self.lower_head()
        print('goal.all_objects_pose', len(goal.all_objects_pose))
        print('goal.all_objects_id', goal.all_objects_id)
        self.get_scene(goal.all_objects_pose, goal.all_objects_id)
        self.move_arm(goal.object_pose)
        p_res = ir_msg.IRPickPlaceResult()
        error_code = 1
        p_res.error_code = error_code
        if error_code != 1:
            self.pick_as.set_aborted(p_res)
        else:
            self.pick_as.set_succeeded(p_res)

    def get_scene(self, all_objects_pose, all_objects_id):
        self.scene = moveit_commander.PlanningSceneInterface()

        rospack = rospkg.RosPack()
        file = rospack.get_path("tiago_iaslab_simulation") + "/meshes/hexagon.dae"

        p = geometry_msgs.PoseStamped()
        p.header.frame_id = "map"
        pose_calc_ = pose_calc('table')
        p.pose.position.x = pose_calc_[0]
        p.pose.position.y = pose_calc_[1]
        self.scene.add_box("pick_table", p, (0.56, 0.56, 0.04))

        name_to_id_cyl = {
            1: 'place_table_b',
            2: 'place_table_g',
            3: 'place_table_r',
        }
        # for cyl in name_to_id_cyl.keys():
        #     p = geometry_msgs.PoseStamped()
        #     p.header.frame_id = "map"
        #     p.pose = pose_calc(str(cyl))
        #     self.scene.add_cylinder("place_table_g", p, 0.21, 0.69)

        name_to_id = {
            1: 'Hexagon',
            2: 'Triangle',
            3: 'cube',
            4: 'Obstacle0',
            5: 'Obstacle1',
            6: 'Obstacle2',
            7: 'Obstacle3',
        }
        for obj_id_index, obj_id in enumerate(all_objects_id):
            p = geometry_msgs.PoseStamped()
            p.header.frame_id = "base_footprint"
            p.pose = all_objects_pose[obj_id_index]
            if obj_id == 3:
                self.scene.add_box("cube", p, (0.05, 0.05, 0.05))
                continue
            self.scene.add_mesh(name_to_id[obj_id], p.pose, file, (1, 1, 1))
        return self.scene

    def move_arm(self, pose_stamped):
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
