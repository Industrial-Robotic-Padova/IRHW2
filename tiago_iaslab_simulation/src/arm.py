
# ! /usr/bin/env python
import rospy
import sys
import copy
from apriltag_ros.msg import AprilTagDetectionArray
from tf import TransformListener
import tf
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
import moveit_commander
import moveit_msgs.msg
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal


class Movement(object):
    def __init__(self):
        self.group = None
        self.object_pose = PoseStamped()
        self.listener = tf.TransformListener()
        self.transform_ok = False
        self.tfBuffer = tf2_ros.Buffer()
        self.tf_l = tf2_ros.TransformListener(self.tfBuffer)
        self.gripper_joint_bounds = dict()
        self.gripper_move_group = ""
        self.gripper_move_group_name = ""
        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_callback)
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
        self.robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("arm_torso")
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory)
        pose_target = geometry_msgs.msg.Pose()
        pose_target.orientation.w = self.object_pose.pose.orientation.w
        pose_target.position.x = self.object_pose.pose.position.x
        pose_target.position.y = self.object_pose.pose.position.y
        pose_target.position.z = self.object_pose.pose.position.z -0.5
        self.group.set_pose_target(self.object_pose)
        plan1 = self.group.plan()

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()

        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan1)
        display_trajectory_publisher.publish(display_trajectory);

        self.move(pose_target)

    def tag_callback(self, msg):
        for detection in msg.detections:
            if detection.id[0] == 3:
                self.object_pose = detection.pose.pose

    # Move the whole arm to the specified pose
    def move(self, pose):
        self.group.set_pose_target(pose)
        return self.group.go()

    # Move the whole arm to the specified state
    def moveToState(self, state):
        self.group.set_joint_value_target(state)
        return self.group.go()

    # Maximize all gripper joints
    def openGripper(self):
        curr_state = self.robot.get_current_state()
        joint_pos = list(curr_state.joint_state.position)
        names = curr_state.joint_state.name
        for i in range(len(names)):
            if names[i] in self.gripper_joint_bounds:
                joint_pos[i] = self.gripper_joint_bounds[names[i]]
        curr_state.joint_state.position = joint_pos
        return self.moveGripperToState(curr_state)

    # Move all joints based on a graspit result
    # and manipulate the scene object
    def grab(self, graspit_result):
        self.attachThis(self.object_to_grasp)
        res = self.moveGripper(graspit_result)
        self.detachThis(self.object_to_grasp)
        return res

    # Move all gripper joints to the specified state
    def moveGripperToState(self, state):
        self.gripper_move_group.set_joint_value_target(state)
        return self.gripper_move_group.go()

    # Shortcut of movegroup's attach_object
    def attachThis(self, object_name):
        touch_links = self.robot.get_link_names(self.gripper_move_group_name)
        self.group.attach_object(object_name, link_name=self.group.get_end_effector_link(), touch_links=touch_links)


if __name__ == '__main__':
    rospy.init_node('pick_aruco_demo')
    Movement()
    rospy.spin()



#
# rospy.init_node('tag_detection_april')
#








