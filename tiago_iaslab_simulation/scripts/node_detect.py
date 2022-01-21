#!/usr/bin/env python

import rospy
import actionlib

import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

from apriltag_ros.msg import AprilTagDetectionArray

import tf
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import PoseStamped

from tiago_iaslab_simulation import msg as ir_msg


g_detects = None


class DetectActionServer:
    def __init__(self):
        self.a_server = actionlib.SimpleActionServer("/ir_detect", ir_msg.IRDetectAction, execute_cb=self.execute_cb, auto_start=False)
        self.a_server.start()

    def execute_cb(self, goal):
        feedback = ir_msg.IRDetectFeedback()
        result = ir_msg.IRDetectResult()
        rate = rospy.Rate(1)

        while g_detects is None:
            print('[DetectActionServer] waiting for apriltag')
            feedback.status = 0
            self.a_server.publish_feedback(feedback)

        result.object_pose = PoseStamped()

        if goal.object_tag in g_detects.keys():
            result.object_pose.pose = g_detects[goal.object_tag].pose.pose.pose
            result.object_pose.header = g_detects[goal.object_tag].pose.header
            feedback.status = 1
            print("[DetectActionServer] detect result:", result)
            result.object_pose = tf_(result.object_pose)
            print("[DetectActionServer] detect result after transform:", result)
            self.a_server.set_succeeded(result)
        else:
            feedback.status = -1
            print("[DetectActionServer] detect result:", 'Not Found')
            self.a_server.set_aborted(result)


def callback_image(img_msg):
    # rospy.loginfo(img_msg.header)
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
        cv2.imshow("Image Window", cv_image)
        cv2.waitKey(3)
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))


def callback_tag(msg):
    # apriltags_ros/AprilTagDetection[] detections
    # AprilTagDetection:
    # int32 id
    # float64 size
    # geometry_msgs/PoseStamped pose
    global g_detects
    g_detects = {i.id[0]: i for i in msg.detections}  # {int: PoseStamped}

    if len(g_detects) != 0:
        print("tag_callback", g_detects.keys())


def tf_(aruco_pose):
    def strip_leading_slash(s):
        return s[1:] if s.startswith("/") else s

    aruco_pose.header.frame_id = strip_leading_slash(aruco_pose.header.frame_id)
    rospy.loginfo("Got: " + str(aruco_pose))

    rospy.loginfo("spherical_grasp_gui: Transforming from frame: " + aruco_pose.header.frame_id + " to 'base_footprint'")
    ps = PoseStamped()
    ps.pose.position = aruco_pose.pose.position
    ps.header.stamp = tfBuffer.get_latest_common_time("base_footprint", aruco_pose.header.frame_id)
    ps.header.frame_id = aruco_pose.header.frame_id
    transform_ok = False
    while not transform_ok and not rospy.is_shutdown():
        try:
            transform = tfBuffer.lookup_transform("base_footprint", ps.header.frame_id, rospy.Time(0))
            aruco_ps = do_transform_pose(ps, transform)
            transform_ok = True
        except tf2_ros.ExtrapolationException as e:
            rospy.logwarn("Exception on transforming point... trying again \n(" + str(e) + ")")
            rospy.sleep(0.01)
            ps.header.stamp = tfBuffer.get_latest_common_time("base_footprint", aruco_pose.header.frame_id)
    return aruco_ps


if __name__ == "__main__":
    rospy.init_node("ir_detect")

    bridge = CvBridge()
    sub_image = rospy.Subscriber("/xtion/rgb/image_raw", Image, callback_image)
    cv2.namedWindow("Image Window", 1)

    sub_msg = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, callback_tag)

    tfBuffer = tf2_ros.Buffer()
    tf_l = tf2_ros.TransformListener(tfBuffer)

    s = DetectActionServer()
    while not rospy.is_shutdown():
        rospy.spin()
