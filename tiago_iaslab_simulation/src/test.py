
# ! /usr/bin/env python
import math

import rospy
import sys
from apriltag_ros.msg import AprilTagDetectionArray
import tf
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
import moveit_commander
from moveit_msgs.msg import RobotState, Constraints, PositionIKRequest
from moveit_msgs.srv import GetPositionIK
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped

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
        self.arm_move_group_name = ""
        self.objects = dict()
        self.pose_factor = 1000
        self.object_to_grasp = ""
        self.gripper_name = ""
        self.planning_srv = None
        self.goal = PoseStamped()
        self.grasp_poses = []
        self.pose_n_joint = dict()
        self.debug = False
        self.already_picked = False
        self.moveit_scene = None
        self.compute_ik_srv = None
        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_callback)
        while not self.transform_ok:
            try:
                transform = self.tfBuffer.lookup_transform('base_footprint', 'tag_3', rospy.Time(0))
                tag_ps = do_transform_pose(self.object_pose, transform)
                self.goal.pose = tag_ps.pose
                print(self.goal.pose)
                self.transform_ok = True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

        self.move_arm()

    def tag_callback(self, msg):
        for detection in msg.detections:
            if detection.id[0] == 3:
                self.object_pose = detection.pose.pose

    def move_arm(self):
        moveit_commander.roscpp_initialize(sys.argv)
        arm = moveit_commander.MoveGroupCommander('arm_torso')
        robot = moveit_commander.RobotCommander()
        rospy.wait_for_service('compute_ik')

        try:
            moveit_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
        except rospy.ServiceException as e:
            rospy.logerror("Service call failed: %s" % e)


        ps = PoseStamped()
        ps.pose.position.x = self.goal.pose.position.x
        ps.pose.position.y = self.goal.pose.position.y
        ps.pose.position.z = self.goal.pose.position.z - 0.4
        ps.pose.orientation.x = 0
        ps.pose.orientation.y =  math.radians(180)
        ps.pose.orientation.z = 0
        ps.pose.orientation.w = 1
        ps.header.frame_id = arm.get_planning_frame()

        req = PositionIKRequest()
        req.group_name = "arm_torso"
        req.pose_stamped = ps
        req.robot_state = robot.get_current_state()
        req.avoid_collisions = False
        req.ik_link_name = arm.get_end_effector_link()
        req.timeout = rospy.Duration(20)
        req.attempts = 0
        print(req)
        resp = moveit_ik(req)
        print(resp)

        arm.set_pose_target(ps)

        plan1 = arm.plan()
        arm.go()
        # ps = PoseStamped()
        # ps.pose.position.x = self.goal.pose.position.x
        # ps.pose.position.y = self.goal.pose.position.y
        # ps.pose.position.z = self.goal.pose.position.z
        # ps.pose.orientation.x = 0
        # ps.pose.orientation.y = 0
        # ps.pose.orientation.z = 0
        # ps.pose.orientation.w = 1
        # ps.header.frame_id = arm.get_planning_frame()
        #
        # req = PositionIKRequest()
        # req.group_name = "arm_torso"
        # req.pose_stamped = ps
        # req.robot_state = robot.get_current_state()
        # print(req.robot_state)
        # req.avoid_collisions = False
        # req.ik_link_name = arm.get_end_effector_link()
        # req.timeout = rospy.Duration(10)
        # req.attempts = 0
        # print(req)
        # resp = moveit_ik(req)
        # print(resp)



if __name__ == '__main__':
    rospy.init_node('pick_aruco_demo')
    Movement()
    rospy.spin()
