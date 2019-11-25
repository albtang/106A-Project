#!/usr/bin/env python
"""
Path Planning Script for project
Author: Parth Baokar, Jasmine Le, Albert Tang, Justin Villamor, Teresa Yang, Valmik Prabhu
"""
import sys
assert sys.argv[1] in ("sawyer", "baxter")
ROBOT = sys.argv[1]

if ROBOT == "baxter":
    from baxter_interface import Limb, gripper as robot_gripper
else:
    from intera_interface import Limb, gripper as robot_gripper

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

    #GRIPPER SETUP AND TEST
    # right_gripper = robot_gripper.Gripper('right_gripper')

    # #Calibrate the gripper (other commands won't work unless you do this first)
    # print('Calibrating...')
    # right_gripper.calibrate()
    # rospy.sleep(2.0)

    # #Open the right gripper
    # print('Opening...')
    # right_gripper.open()
    # rospy.sleep(1.0)
    # print('Gripper!')

    # ##
    # ## Add the obstacle to the planning scene here
    ### SHOULD WE ADD PLACED PIECES AS OBSTACLES ????
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
    actual = ["L", "O", "S", "Z"]

    # Get the desired position and orientation of each piece by parsing the desired string
    # goals = raw_input("Enter in a 2D matrix of your desired layout, with each element separated by spaces and each row separated by periods: ")
    goals = "L O O G S S.L O O S S Z Z.L L I I I I Z Z"
    while goals:
        goals = goals.split(".")
        for row_i in range(len(goals)):
            goals[row_i] = goals[row_i].split()
        # goals = [[None, "L", "L"], ["O","O", "L"], ["O","O", "L"]]
        goals = decipher_final_configuration(goals)

        raw_input("Press <Enter> to move the right arm to pick up a piece: ")
        while not rospy.is_shutdown() and len(actual) > 0:
            # Booleans to track state
            in_hand, picked, placed = False, False, False
            piece = actual.pop(0)

            ### PICKING UP PIECE FROM ACTUAL LOCATION ###
            while not rospy.is_shutdown() and not picked:
                try:
                    # Pick up the piece in the original position
                    if ROBOT == "baxter":
                        x, y, z = 0.47, -0.85, 0.07
                    else:
                        x, y, z = 0.8, 0.05, -0.23
                    
                    original = PoseStamped()
                    original.header.frame_id = "base"

                    # x, y, and z position
                    original.pose.position.x = x
                    original.pose.position.y = y
                    original.pose.position.z = z
                    # Orientation as a quaternion
                    original.pose.orientation.x = 0.0
                    original.pose.orientation.y = -1.0
                    original.pose.orientation.z = 0.0
                    original.pose.orientation.w = 0.0

                    if not in_hand:
                        # Translate over such that the piece is above the actual position
                        original.pose.position.z += .2
                        plan = planner.plan_to_pose(original, [orien_const])
                        if not planner.execute_plan(plan):
                            # Reset position of z such that if the loop fails, we haven't mutated it
                            original.pose.position.z -= .2
                            raise Exception("Execution failed")

                        # Actually pick up the piece
                        original.pose.position.z -= .2
                        plan = planner.plan_to_pose(original, [orien_const])
                        if not planner.execute_plan(plan):
                            raise Exception("Execution failed")
                        ## GRIPPER OPEN (succ)
                        in_hand = True

                    # Raise the arm a little bit so that other pieces are not affected
                    original.pose.position.z += .2
                    plan = planner.plan_to_pose(original, [orien_const])
                    if not planner.execute_plan(plan):
                        # Reset position of z such that if the loop fails, we haven't mutated it
                        original.pose.position.z -= .2
                        raise Exception("Execution failed")

                    picked = True

                except Exception as e:
                    print e
                    traceback.print_exc()
                else:
                    break

            ### PLACING PIECE AT DESIRED LOCATION ###
            text = raw_input("Press <Enter> to move the right arm to place the piece or 'back' to redo the previous step: ")
            if text is "back":
                picked = False

            while not rospy.is_shutdown() and picked and not placed:

                try:
                    goal = goals.get(piece)[0]
                    print(goal)

                    if in_hand:
                        # Translate over such that the piece is above the desired position
                        goal.pose.position.z += .2
                        plan = planner.plan_to_pose(goal, [orien_const])
                        if not planner.execute_plan(plan):
                            # Reset position of z such that if the loop fails, we haven't mutated it
                            goal.pose.position.z -= .2
                            raise Exception("Execution failed")

                        # Actually place the piece on table
                        goal.pose.position.z -= .2
                        plan = planner.plan_to_pose(goal, [orien_const])
                        if not planner.execute_plan(plan):
                            raise Exception("Execution failed")
                        ## GRIPPER CLOSE

                        in_hand = False

                    # Raise the arm a little bit so that other pieces are not affected
                    goal.pose.position.z += .2
                    plan = planner.plan_to_pose(goal, [orien_const])
                    if not planner.execute_plan(plan):
                        # Reset position of z such that if the loop fails, we haven't mutated it
                        goal.pose.position.z -= .2
                        raise Exception("Execution failed")

                    temp = goals.get(piece).pop(0)
                    placed = True
                    picked = False

                except Exception as e:
                    print e
                    traceback.print_exc()
                else:
                    break
                
            raw_input("Press <Enter> to move onto the next piece or 'back': ")
            if raw_input is "back":
                placed = False
                picked = True
                goals.insert(0, temp)
        
        print("We have finished solving the puzzle. If you would like to continue: ")
        goals = None
        # goals = raw_input("Enter in a 2D matrix of your desired layout, with each element separated by spaces and each row separated by periods: ")

if __name__ == '__main__':
    rospy.init_node('moveit_node')
    main()
