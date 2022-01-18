
# ! /usr/bin/env python
import rospy
import sys
import time
from apriltag_ros.msg import AprilTagDetectionArray
import tf
from geometry_msgs.msg import PoseStamped
import moveit_commander
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
from moveit_msgs.msg import CollisionObject
from moveit_msgs.srv import GetPositionIKRequest, GraspPlanning, GetPositionIK


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
                print(transform)
                tag_ps = do_transform_pose(self.object_pose, transform)
                print(tag_ps)
                self.transform_ok = True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.group = moveit_commander.MoveGroupCommander("arm_torso")

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

    # Move all joints based on a graspit result
    def moveGripper(self, graspit_result):
        curr_state = self.robot.get_current_state()
        joint_pos = list(curr_state.joint_state.position)
        names = curr_state.joint_state.name
        for i in range(len(graspit_result.joint_names)):
            for j in range(len(names)):
                if graspit_result.joint_names[i] == names[j]:
                    joint_pos[j] = self.gripper_joint_bounds[names[j]] - abs(
                        graspit_result.points[0].positions[i] / self.pose_factor)
                    break
        curr_state.joint_state.position = joint_pos
        return self.moveGripperToState(curr_state)

    # Move all gripper joints to the specified state
    def moveGripperToState(self, state):
        self.gripper_move_group.set_joint_value_target(state)
        return self.gripper_move_group.go()

    # Shortcut of tf's lookup_transform
    def lookupTF(self, target_frame, source_frame):
        return self.tfBuffer.lookup_transform(target_frame, source_frame, rospy.Time(), rospy.Duration(10))

    # Call graspit for the specified object
    def graspThis(self, object_name):
        target = CollisionObject()
        target.id = str(self.objects[object_name][0])
        target.primitive_poses = [self.objects[object_name][1].pose]
        response = self.planning_srv(group_name=self.gripper_name, target=target)
        return response.grasps

    # Shortcut of movegroup's attach_object
    def attachThis(self, object_name):
        touch_links = self.robot.get_link_names(self.gripper_move_group_name)
        self.group.attach_object(object_name, link_name=self.group.get_end_effector_link(),
                                          touch_links=touch_links)

    # Shortcut of movegroup's detach_object
    def detachThis(self, object_name):
        self.group.detach_object(object_name)

    # Pick and place!
    def uberPlan(self):
        return self.pick() and self.place()

    # Open the gripper, move the arm to the grasping pose
    # and grab the object
    def pick(self):
        if not self.already_picked:
            # GraspIt assumes maxed out joints, so that's what we do here
            self.openGripper()
            time.sleep(1)
            valid_g = self.discard(self.grasp_poses)

            if len(valid_g) > 0:
                for j in range(len(valid_g[0])):
                    self.group.set_start_state_to_current_state()
                    if self.move(valid_g[0][j].pose):
                        time.sleep(1)
                        return self.grab(self.pose_n_joint[valid_g[0][j]])
                self.error_info = "Error while trying to pick the object!"
            else:
                self.error_info = "No valid grasps were found!"
            return False
        return True

    def discard(self, poses):
        validp = []
        validrs = []
        req = GetPositionIKRequest()
        req.ik_request.group_name = self.arm_move_group_name
        req.ik_request.robot_state = self.robot.get_current_state()
        req.ik_request.avoid_collisions = True
        for p in poses:
            req.ik_request.pose_stamped = p
            if self.debug:
                br = tf.TransformBroadcaster()
                br.sendTransform((p.pose.position.x, p.pose.position.y, p.pose.position.z), (
                p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w),
                                 rospy.Time.now(), "candidate_grasp_pose", p.header.frame_id)
            k = self.compute_ik_srv(req)
            if k.error_code.val == 1:
                validp.append(p)
                validrs.append(k.solution)
        if validp:
            return [validp, validrs]
        return []


if __name__ == '__main__':
    rospy.init_node('pick_aruco_demo')
    Movement()
    rospy.spin()

