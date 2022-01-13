#! /usr/bin/env python
import rospy
import time
import actionlib
from  apriltag_ros.msg import AprilTagDetectionArray


def tag_callback(msg):
    print('ID:',  msg.detections[0].id)
    print('FINAL POSES: ', msg.detections[0].pose.pose.pose.position)


rospy.init_node('tag_detection_april')
tag_msg = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, tag_callback)
rospy.spin()
