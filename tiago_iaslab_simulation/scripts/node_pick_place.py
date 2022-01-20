# ! /usr/bin/env python
# -*- coding: utf-8 -*-
import copy

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from actionlib import SimpleActionClient, SimpleActionServer

from tiago_iaslab_simulation import msg as ir_msg


class PickAndPlaceServer(object):

    def __init__(self):
        rospy.loginfo("Setting publishers to torso and head controller...")
        self.torso_cmd = rospy.Publisher('/torso_controller/command', JointTrajectory, queue_size=1)
        self.head_cmd = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=1)

        rospy.loginfo("Waiting for '/play_motion' AS...")
        self.play_m_as = SimpleActionClient('/play_motion', PlayMotionAction)
        if not self.play_m_as.wait_for_server(rospy.Duration(20)):
            rospy.logerr("Could not connect to /play_motion AS")
            exit()

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


if __name__ == '__main__':
    rospy.init_node('ir_pick_place')
    sphere = PickAndPlaceServer()
    rospy.spin()
