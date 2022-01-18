
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


class MoveArm(object):
    def __init__(self):
        self.group = None
        self.object_pose = PoseStamped()
        self.listener = tf.TransformListener()
        self.transform_ok = False
        self.tfBuffer = tf2_ros.Buffer()
        self.tf_l = tf2_ros.TransformListener(self.tfBuffer)

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
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group = moveit_commander.MoveGroupCommander("arm_torso")
        display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path',
            moveit_msgs.msg.DisplayTrajectory)
        pose_target = geometry_msgs.msg.Pose()
        pose_target.orientation.w = tag_ps.pose.orientation.w
        pose_target.position.x = tag_ps.pose.position.x
        pose_target.position.y = tag_ps.pose.position.y
        pose_target.position.z = tag_ps.pose.position.z
        print("============ Reference frame: %s" % group.get_planning_frame())

        ## We can also print the name of the end-effector link for this group
        print("============ Reference frame: %s" % group.get_end_effector_link())
        # pose_target = geometry_msgs.msg.Pose()
        # pose_target.orientation.w = 1.0
        # pose_target.position.x = 0.7
        # pose_target.position.y = -0.05
        # pose_target.position.z = 1.1
        group.set_pose_target(pose_target)
        plan1 = group.plan()
        rospy.sleep(5)
        print("============ Visualizing plan1")
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan1)
        display_trajectory_publisher.publish(display_trajectory);
        print("============ Waiting while plan1 is visualized (again)...")
        rospy.sleep(5)
        waypoints = []

        # start with the current pose
        waypoints.append(group.get_current_pose().pose)

        # first orient gripper and move forward (+x)
        wpose = geometry_msgs.msg.Pose()
        wpose.orientation.w = 1.0
        wpose.position.x = waypoints[0].position.x + 0.1
        wpose.position.y = waypoints[0].position.y
        wpose.position.z = waypoints[0].position.z
        waypoints.append(copy.deepcopy(wpose))

        # second move down
        wpose.position.z -= 0.10
        waypoints.append(copy.deepcopy(wpose))

        # third move to the side
        wpose.position.y += 0.05
        waypoints.append(copy.deepcopy(wpose))

        ## We want the cartesian path to be interpolated at a resolution of 1 cm
        ## which is why we will specify 0.01 as the eef_step in cartesian
        ## translation.  We will specify the jump threshold as 0.0, effectively
        ## disabling it.
        (plan3, fraction) = group.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.01,  # eef_step
            0.0)  # jump_threshold

        group.go()

    def tag_callback(self, msg):
        for detection in msg.detections:
            if detection.id[0] == 3:
                self.object_pose = detection.pose.pose


if __name__ == '__main__':
    rospy.init_node('pick_aruco_demo')
    MoveArm()
    rospy.spin()
