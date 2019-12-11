#!/usr/bin/env python
import sys
assert sys.argv[1] in ("sawyer", "baxter")
ROBOT = sys.argv[1]

if ROBOT == "baxter":
    from baxter_interface import Limb
else:
    from intera_interface import Limb

import rospy
import numpy as np
import traceback

from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import PoseStamped

from path_planner import PathPlanner

# import rospy
# from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
# from geometry_msgs.msg import PoseStamped
# from moveit_commander import MoveGroupCommander
# import numpy as np
# from numpy import linalg
# import sys
# from baxter_interface import gripper as robot_gripper
# from intera_interface import gripper as robot_gripper

def main():
    # right_hand_camera values
    # - Translation: [0.403, 0.288, 0.225]
    # - Rotation: in Quaternion [0.424, 0.867, -0.133, 0.228]
    #             in RPY (radian) [-3.098, 0.532, 2.244]
    #             in RPY (degree) [-177.491, 30.484, 128.585]

    planner = PathPlanner("left_arm")
    limb = Limb("left")
    while not rospy.is_shutdown():
        while not rospy.is_shutdown():
            try:
                if ROBOT == "baxter":
                    x, y, z = 0.403, 0.288, 0.225
                else:
                    x, y, z = 0.8, 0.05, -0.23
                goal_1 = PoseStamped()
                goal_1.header.frame_id = "base"

                # x, y, z = -.4, -0.85, 0.07

                #x, y, and z position
                goal_1.pose.position.x = x
                goal_1.pose.position.y = y
                goal_1.pose.position.z = z

                #Orientation as a quaternion
                goal_1.pose.orientation.x = 0.0
                goal_1.pose.orientation.y = -1.0
                goal_1.pose.orientation.z = 0.0
                goal_1.pose.orientation.w = 0.0

                # Might have to edit this . . . 
                plan = planner.plan_to_pose(goal_1, [])
                # plan = planner.plan_to_pose(goal_1, [orien_const])

                raw_input("Press <Enter> to move the right arm to goal pose 1: ")
                # if not planner.execute_plan(plan):
                if not planner.execute_plan(plan):
                    raise Exception("Execution failed")
            except Exception as e:
                print e
                traceback.print_exc()
            else:
                break

#Python's syntax for a main() method
if __name__ == '__main__':
    rospy.init_node('moveit_node')
    main()

