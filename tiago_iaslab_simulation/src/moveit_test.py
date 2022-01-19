
# ! /usr/bin/env python
import rospy
import sys
import time
import copy
import geometry_msgs
from apriltag_ros.msg import AprilTagDetectionArray
import tf
from geometry_msgs.msg import PoseStamped
import moveit_commander
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
from moveit_msgs.msg import CollisionObject
from moveit_msgs.srv import GetPositionIKRequest, GraspPlanning, GetPositionIK
import moveit_msgs.msg
from math import pi


class MoveArm(object):
    def __init__(self):
        self.group = None
        self.object_pose = PoseStamped()
        self.listener = tf.TransformListener()
        self.transform_ok = False
        self.tfBuffer = tf2_ros.Buffer()
        self.tf_l = tf2_ros.TransformListener(self.tfBuffer)
        while not self.transform_ok:
            try:
                transform = self.tfBuffer.lookup_transform('base_footprint', 'tag_3', rospy.Time(0))
                print(transform)
                tag_ps = do_transform_pose(self.object_pose, transform)
                print(tag_ps)
                self.transform_ok = True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        moveit_commander.roscpp_initialize(sys.argv)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group = moveit_commander.MoveGroupCommander("arm_torso")
        display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path',
            moveit_msgs.msg.DisplayTrajectory)
        
        print(robot.get_current_state())
        joint_goal = group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -pi / 4
        joint_goal[2] = 0
        joint_goal[3] = -pi / 2
        joint_goal[4] = 0
        joint_goal[5] = pi / 3
        joint_goal[6] = 0
        group.go(joint_goal, wait=True)
        group.stop()
        print("============ Visualizing plan1")
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan1)
        display_trajectory_publisher.publish(display_trajectory)
        print("============ Waiting while plan1 is visualized (again)...")
        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_callback)
        pose_target = geometry_msgs.msg.Pose()
        pose_target.orientation.w = -tag_ps.pose.orientation.w
        pose_target.position.x = tag_ps.pose.position.x
        pose_target.position.y = tag_ps.pose.position.y
        pose_target.position.z = tag_ps.pose.position.z - 0.5
        print(tag_ps.pose.position.x, tag_ps.pose.position.y, tag_ps.pose.position.z - 0.5)
        print("============ Reference frame: %s" % group.get_planning_frame())
        print("============ Reference frame: %s" % group.get_end_effector_link())
        group.set_pose_target(pose_target)
        plan = group.go(wait=True)
        group.stop()
        group.clear_pose_targets()
        pose_target = geometry_msgs.msg.Pose()
        pose_target.orientation.w = -tag_ps.pose.orientation.w
        pose_target.position.x = tag_ps.pose.position.x
        pose_target.position.y = tag_ps.pose.position.y
        pose_target.position.z = tag_ps.pose.position.z
        print("============ Reference frame: %s" % group.get_planning_frame())
        print("============ Reference frame: %s" % group.get_end_effector_link())
        group.set_pose_target(pose_target)
        plan = group.go(wait=True)
        group.stop()
        group.clear_pose_targets()

    def tag_callback(self, msg):
        for detection in msg.detections:
            if detection.id[0] == 3:
                self.object_pose = detection.pose.pose


if __name__ == '__main__':
    rospy.init_node('pick_aruco_demo')
    MoveArm()
    rospy.spin()
