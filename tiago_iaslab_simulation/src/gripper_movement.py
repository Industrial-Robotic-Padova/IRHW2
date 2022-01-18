# !/usr/bin/env python
import rospy
import tf2_ros
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from actionlib import SimpleActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from apriltag_ros.msg import AprilTagDetectionArray
import tf
import geometry_msgs
import moveit_msgs
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, TransformStamped


# Point the head using controller

class PointHeadClient(object):
    def __init__(self, msg):
        self.tf2_buffer = tf2_ros.Buffer()
        self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer)
        rospy.loginfo("Waiting for '/play_motion' AS...")
        self.head_cmd = rospy.Publisher(
            '/head_controller/command', JointTrajectory, queue_size=1.0)
        self.torso_cmd = rospy.Publisher(
            '/torso_controller/command', JointTrajectory, queue_size=1.0)
        self.arm_cmd = rospy.Publisher(
            '/arm_controller/command', JointTrajectory, queue_size=1.0)
        self.gripper_cmd = rospy.Publisher("/gripper_controller/command", JointTrajectory, queue_size=1.0)
        rospy.loginfo("Moving head down")
        self.play_m_as = SimpleActionClient('/play_motion', PlayMotionAction)
        if not self.play_m_as.wait_for_server(rospy.Duration(20)):
            rospy.logerr("Could not connect to /play_motion AS")
            exit()
        rospy.loginfo("Connected!")
        rospy.sleep(1.0)
        rospy.loginfo("Done initializing PickAruco.")
        pmg = PlayMotionGoal()
        pmg.motion_name = 'pick_final_pose'
        pmg.skip_planning = False
        self.play_m_as.send_goal_and_wait(pmg)
        rospy.loginfo("Raise object done.")
        self.object = geometry_msgs.msg.PoseStamped()
        self.msg = msg
        self.moves()
        self.gripper_pose = geometry_msgs.msg.PoseStamped()
        

    def moves(self):
        #self.move_head()
        #self.move_torso()
        #self.open_arm()
        self.move_gripper()
        #self.move_group_arm()

    def lookupTF(self, target_frame, source_frame):
        return self.tf2_buffer.lookup_transform(target_frame, source_frame, rospy.Time(), rospy.Duration(10))

    def move_gripper(self):
        pose_factor = 1000
        self.object.header.frame_id = self.msg.header.frame_id
        self.object.pose.position = self.msg.detections[0].pose.pose.pose.position
        self.object.pose.orientation = self.msg.detections[0].pose.pose.pose.orientation
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = self.object.header.frame_id
        transform.transform.translation.x = self.object.pose.position.x
        transform.transform.translation.y = self.object.pose.position.y
        transform.transform.translation.z = self.object.pose.position.z
        transform.transform.rotation.x = self.object.pose.orientation.x
        transform.transform.rotation.y = self.object.pose.orientation.y
        transform.transform.rotation.z = self.object.pose.orientation.z
        transform.transform.rotation.w = self.object.pose.orientation.w
        self.tf2_buffer.set_transform(transform, "/base_link")
        print(self.tf2_buffer)

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
        jt.joint_names = ['arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 'arm_5_joint', 'arm_6_joint',
                          'arm_7_joint']
        jtp = JointTrajectoryPoint()
        #jtp.positions = [2.5, 0.2, -2.1, 1.9, 1.0, -0.5, 0.0]
        jtp.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        jtp.time_from_start = rospy.Duration(2.0)
        jt.points.append(jtp)
        self.arm_cmd.publish(jt)
        rospy.loginfo("Done.")

    def move_group_arm(self):
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group = moveit_commander.MoveGroupCommander("left_arm")
        display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path',
            moveit_msgs.msg.DisplayTrajectory)
        print("============ Waiting for RVIZ...")
        rospy.sleep(10)
        print("============ Starting tutorial ")
        print("============ Reference frame: %s" % group.get_planning_frame())
        print("============ Reference frame: %s" % group.get_end_effector_link())
        print("============ Robot Groups:")
        print(robot.get_group_names())
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("============")
        print("============ Generating plan 1")
        group.set_pose_target(self.gripper_pose)

        plan1 = group.plan()
        print("============ Waiting while RVIZ displays plan1...")
        rospy.sleep(5)
        print("============ Visualizing plan1")
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan1)
        display_trajectory_publisher.publish(display_trajectory);
        print("============ Waiting while plan1 is visualized (again)...")
        rospy.sleep(5)


def tag_callback(msg):
    if msg.detections[0].id[0] == 3:
        head_action = PointHeadClient(msg)


if __name__ == "__main__":
    # Create a node
    rospy.init_node("demo")
    tag_msg = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, tag_callback)
    # Setup clients

    rospy.spin()

