
# ! /usr/bin/env python
import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from tf import TransformListener
import tf
import geometry_msgs
from geometry_msgs.msg import PoseStamped
import moveit_commander
import tf2_ros
from tf2_geometry_msgs import do_transform_pose


class Movement(object):
    def __init__(self):
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

    def tag_callback(self, msg):
        for detection in msg.detections:
            if detection.id[0] == 3:
                self.object_pose = detection.pose.pose

if __name__ == '__main__':
    rospy.init_node('pick_aruco_demo')
    Movement()
    rospy.spin()



#
# rospy.init_node('tag_detection_april')
#








