#!/usr/bin/env python
"""
Path Planning Script for project
Author: Parth Baokar, Jasmine Le, Albert Tang, Justin Villamor, Teresa Yang, Valmik Prabhu
"""
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
# from controller import Controller
from final_calculation import decipher_final_configuration

def main():
    """
    Main Script
    """

    planner = PathPlanner("right_arm")

    # ##
    # ## Add the obstacle to the planning scene here
    # ##
    # pose = PoseStamped()
    # pose.header.frame_id = "base"
    # pose.pose.position.x = .5
    # pose.pose.position.y = 0.0
    # pose.pose.position.z = -.2
    # pose.pose.orientation.x = 0.0
    # pose.pose.orientation.y = 0.0
    # pose.pose.orientation.z = 0.0
    # pose.pose.orientation.w = 1.0
    # planner.add_box_obstacle(np.ndarray(shape=(3,), buffer=np.array([.4, 1.2, .1])), "table", pose)
    # pose = PoseStamped()
    # pose.header.frame_id = "base"
    # pose.pose.position.x = -.5
    # pose.pose.position.y = 0.0
    # pose.pose.position.z = 0.0
    # pose.pose.orientation.x = 0.0
    # pose.pose.orientation.y = 0.0
    # pose.pose.orientation.z = 0.0
    # pose.pose.orientation.w = 1.0
    # planner.add_box_obstacle(np.ndarray(shape=(3,), buffer=np.array([.1, 3, 3])), "wall", pose)

    # #Create a path constraint for the arm
    orien_const = OrientationConstraint()
    orien_const.link_name = "right_gripper";
    orien_const.header.frame_id = "base";
    orien_const.orientation.y = -1.0;
    orien_const.absolute_x_axis_tolerance = 0.1;
    orien_const.absolute_y_axis_tolerance = 0.1;
    orien_const.absolute_z_axis_tolerance = 0.1;
    orien_const.weight = 1.0;

    # Get the initial position and orientation of each piece ###PARTH"S FUNCTION
    actual = []

    # Get the desired position and orientation of each piece
    # goals = raw_input("Enter in a 2D matrix of your desired layout: ")
    goals = [["O","O"], ["O","O"]]
    goals = decipher_final_configuration(goals)

    while not rospy.is_shutdown():
        picked, placed = False, False
        # piece = actual.pop(0)
        piece = "O"

        # # Start at neutral position
        # x, y, z = 0.5, -0.75, 0.3
        # neutral = PoseStamped()
        # neutral.header.frame_id = "base"

        # #x, y, and z position
        # neutral.pose.position.x = x
        # neutral.pose.position.y = y
        # neutral.pose.position.z = z

        # #Orientation as a quaternion
        # neutral.pose.orientation.x = 0.0
        # neutral.pose.orientation.y = -1.0
        # neutral.pose.orientation.z = 0.0
        # neutral.pose.orientation.w = 0.0
        # neutral.pose.position.z += .2
        # plan = planner.plan_to_pose(neutral, [orien_const])

        # if not planner.execute_plan(plan):
        #     raise Exception("Execution failed")

        ### ACTUAL LOCATION ###
        while not rospy.is_shutdown() and not picked:
            try:
                # Pick up the piece in the original position
                if ROBOT == "baxter":
                    x, y, z = 0.47, -0.85, 0.07
                # else:
                    # x, y, z = 0.8, 0.05, -0.23
                
                original = PoseStamped()
                original.header.frame_id = "base"

                #x, y, and z position
                original.pose.position.x = x
                original.pose.position.y = y
                original.pose.position.z = z

                #Orientation as a quaternion
                original.pose.orientation.x = 0.0
                original.pose.orientation.y = -1.0
                original.pose.orientation.z = 0.0
                original.pose.orientation.w = 0.0

                plan = planner.plan_to_pose(original, [orien_const])

                raw_input("Press <Enter> to move the right arm to pick up a piece: ")
                if not planner.execute_plan(plan):
                    raise Exception("Execution failed")

                # Raise the arm a little bit so that other pieces are not affected
                original.pose.position.z += .2
                plan = planner.plan_to_pose(original, [orien_const])
                if not planner.execute_plan(plan):
                    raise Exception("Execution failed")

                picked = True

            except Exception as e:
                print e
                traceback.print_exc()
            else:
                break

        ### DESIRED LOCATION ###
        while not rospy.is_shutdown() and picked and not placed:

            try:
                # Translate over such that the piece is above the desired position
                goal = goals.get(piece)[0]
                goal.pose.position.z += .2
                plan = planner.plan_to_pose(goal, [orien_const])
                print(goal)

                raw_input("Press <Enter> to move the right arm to place the piece: ")
                if not planner.execute_plan(plan):
                    goal.pose.position.z -= .2
                    raise Exception("Execution failed")
                print("A")

                # Actually place the piece on table
                goal.pose.position.z -= .2

                print(goal)
                plan = planner.plan_to_pose(goal, [orien_const])
                if not planner.execute_plan(plan):
                    raise Exception("Execution failed")
                print("B")

                # goals.get(piece).pop(0)
                placed = True
                picked = False

            except Exception as e:
                print e
            else:
                break

if __name__ == '__main__':
    rospy.init_node('moveit_node')
    main()
