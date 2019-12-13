#!/usr/bin/env python
import tf
import tf2_ros
import sys
import rospy
import numpy as np
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Pose
 
AR_TRACKER_TOPIC = '/ar_pose_marker'
ACTUAL_POSE_TOPIC = '/actual_ar_pose'

#Define the method which contains the node's main functionality
def listener():
    rospy.init_node('listener', anonymous=True)
    target_frame = 'base'
    source_frame = 'left_hand_camera'

    ar_msg = rospy.wait_for_message(AR_TRACKER_TOPIC, AlvarMarkers)
    base_publisher = rospy.Publisher(ACTUAL_POSE_TOPIC, Pose, queue_size=10)

    tfBuffer = tf2_ros.Buffer()

    while True:
        tfListener = tf2_ros.TransformListener(tfBuffer)
        try:
            trans = tfBuffer.lookup_transform(target_frame, source_frame, rospy.Time())
            tf1 = tf.TransformerROS()
            translation = trans.transform.translation
            translation = (translation.x, translation.y, translation.z)
            rotation = trans.transform.rotation
            rotation = (rotation.x, rotation.y, rotation.z, rotation.w)
            transformation_ar = tf1.fromTranslationRotation(translation, rotation)
            break
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print(e)

        rospy.sleep(1)
    
    ar_loc = np.array([ar_msg.markers[0].pose.pose.position.x, ar_msg.markers[0].pose.pose.position.y, ar_msg.markers[0].pose.pose.position.z, 1])
    actual_ar_base = np.dot(transformation_ar, ar_loc)
    
    
    pose_to_publish = Pose()
    pose_to_publish.position.x = actual_ar_base[0]
    pose_to_publish.position.y = actual_ar_base[1]
    pose_to_publish.position.z = actual_ar_base[2]
    pose_to_publish.orientation = ar_msg.markers[0].pose.pose.orientation

    while not rospy.is_shutdown():
        base_publisher.publish(pose_to_publish)


if __name__ == '__main__':
    listener()