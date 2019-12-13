#!/usr/bin/env python
import tf
import tf2_ros
import sys
import rospy
import numpy as np
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Pose
 
AR_TRACKER_TOPIC = '/ar_pose_marker'
ACTUAL_FUCKING_TOPIC = '/actual_fucking_marker'

#Define the method which contains the node's main functionality
def listener():
    rospy.init_node('listener', anonymous=True)
    target_frame = 'base'
    source_frame = 'left_hand_camera'

    ar_msg = rospy.wait_for_message(AR_TRACKER_TOPIC, AlvarMarkers)
    base_publisher = rospy.Publisher(ACTUAL_FUCKING_TOPIC, Pose, queue_size=10)

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
    
#  #!/usr/bin/env python
# import sys
# import tf2_ros
# import rospy
# from sensor_msgs.msg import Image
 
# AR_TRACKER_TOPIC = '/ar_pose_marker'
# ACTUAL_FUCKING_TOPIC = '/actual_fucking_marker'

# def ar_callback(ar_msg):
#     [source_frame, target_frame] = sys.argv[1:3]
#     # target_frame = 'base'
#     # source_frame = 'left_hand_camera'
#     tfBuffer = tf2_ros.Buffer()

#     while True:
#         tfListener = tf2_ros.TransformListener(tfBuffer)
#         try:
#             trans = tfBuffer.lookup_transform(target_frame, source_frame, rospy.Time())
#             print(tfListener.fromTranslationRotation(trans.translation, trans.rotation))
            
#         except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
#             print(e)

#         rospy.sleep(1)


# def main():
#     rospy.init_node('asimov_is_dumb', anonymous=True)
#     ar_subscriber = rospy.Subscriber(AR_TRACKER_TOPIC, AlvarMarkers, ar_callback)
#     base_publisher = rospy.Publisher(ACTUAL_FUCKING_TOPIC, Pose, queue_size=10)
#     rospy.spin()
#     cv2.destroyAllWindows()

# if __name__ == '__main__':
#     main()