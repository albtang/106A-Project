#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
import numpy as np
from numpy import linalg
import sys
from baxter_interface import gripper as robot_gripper
# from intera_interface import gripper as robot_gripper

def main(robo):
    #Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    rospy.init_node('service_query')
    arm = 'left'
    #Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)

    while not rospy.is_shutdown():
        raw_input('Press [ Enter ]: ')
        
        #Construct the request
        request = GetPositionIKRequest()
        request.ik_request.group_name = arm + "_arm"

        # request.ik_request.ik_link_name = link
        request.ik_request.attempts = 20
        request.ik_request.pose_stamped.header.frame_id = "base"

        # # right_hand_camera values
        # # - Translation: [0.403, 0.288, 0.225]
        # # - Rotation: in Quaternion [0.424, 0.867, -0.133, 0.228]
        # #             in RPY (radian) [-3.098, 0.532, 2.244]
        # #             in RPY (degree) [-177.491, 30.484, 128.585]
        
        initial_pos = [0.403, 0.288, 0.255, 0.424, 0.867, -0.133, 0.228]

        request.ik_request.pose_stamped.pose.position.x = initial_pos[0]
        request.ik_request.pose_stamped.pose.position.y = initial_pos[1]
        request.ik_request.pose_stamped.pose.position.z = initial_pos[2]
        request.ik_request.pose_stamped.pose.orientation.x = initial_pos[3]
        request.ik_request.pose_stamped.pose.orientation.y = initial_pos[4]
        request.ik_request.pose_stamped.pose.orientation.z = initial_pos[5]
        request.ik_request.pose_stamped.pose.orientation.w = initial_pos[6]
            
        try:
            #Send the request to the service
            response = compute_ik(request)

            #Print the response HERE
            print(response)
            group = MoveGroupCommander(arm + "_arm")

            # Setting position and orientation target
            group.set_pose_target(request.ik_request.pose_stamped)

            # TRY THIS
            # Setting just the position without specifying the orientation
            # group.set_position_target([0.5, -0.5, 0.0])

            # Plan IK and execute
            group.go()

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

#Python's syntax for a main() method
if __name__ == '__main__':
    main(sys.argv[1])

