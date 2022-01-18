#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from apriltag_ros.msg import AprilTagDetectionArray


def image_callback(img_msg):
    rospy.loginfo(img_msg.header)

    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
        cv2.imshow("Image Window", cv_image)
        cv2.waitKey(3)
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))


def tag_callback(msg):
    print(msg)
    print('FINAL POSES: ', msg.detections[0].pose.pose.pose.position)


if __name__ == "__main__":
    rospy.init_node('camera', anonymous=True)
    rospy.loginfo("node camera stating...")
    bridge = CvBridge()

    sub_image = rospy.Subscriber("/xtion/rgb/image_raw", Image, image_callback)
    sub_msg = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, tag_callback)

    cv2.namedWindow("Image Window", 1)

    while not rospy.is_shutdown():
        rospy.spin()
