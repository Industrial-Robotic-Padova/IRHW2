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

        sub_image = rospy.Subscriber("/xtion/rgb/image_raw", Image, callback_image)
        sub_msg = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, callback_tag)
        cv2.namedWindow("Image Window", 1)

        self.a_server = actionlib.SimpleActionServer("/ir_detect", ir_msg.IRDetectAction, execute_cb=self.execute_cb, auto_start=False)
        self.a_server.start()

    def execute_cb(self, goal):
        feedback = ir_msg.IRDetectFeedback()
        result = ir_msg.IRDetectResult()
        rate = rospy.Rate(1)

        while g_detects is None:
            print('[Feedback] waiting for apriltag')
            feedback.status = 0
            self.a_server.publish_feedback(feedback)

        if goal.object_tag in g_detects.keys():
            result.position = g_detects[goal.object_tag]
            feedback.status = 1
        result.position = []
        feedback.status = -1
        print("[Result] detect result:", result)
        self.a_server.set_succeeded(result)


def callback_image(img_msg):
    rospy.loginfo(img_msg.header)

    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
        cv2.imshow("Image Window", cv_image)
        cv2.waitKey(3)
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))


def callback_tag(msg):
    global g_detects
    g_detects = {i.id: i.pose.pose for i in msg.detections}
    print('tag_callback', g_detects)


def tf_callback_(object_pose: PoseStamped):
    tfBuffer = tf2_ros.Buffer()

    while True:
        try:
            transform = tfBuffer.lookup_transform('base_footprint', 'tag_3', rospy.Time(0))
            print(transform)
            tag_ps = do_transform_pose(object_pose, transform)
            print(tag_ps)
            return tag_ps
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue


if __name__ == "__main__":
    bridge = CvBridge()

    rospy.init_node("ir_detect")
    s = DetectActionServer()
    rospy.spin()
