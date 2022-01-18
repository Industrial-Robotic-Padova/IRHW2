
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

rospy.init_node('tag_detection_april')
listener = tf.TransformListener()
while not rospy.is_shutdown():
        try:
            x = listener.lookupTransform('map', 'tag_3', rospy.Time(0))
            print(x)
            break
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue



