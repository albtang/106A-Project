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
from geometry_msgs.msg import PoseStamped, PoseArray

from path_planner import PathPlanner
# from controller import Controller
from final_calculation import decipher_final_configuration

# Ros Set Up
NODE = 'moveit_node'
OBJECTS_SUB_TOPIC = "detected_objects"
rospy.init_node(NODE)

#TODO: add the rest of the colors
ColorMapping = {
    "blue" : "O",
    "red" : "L",
    "purple" : "J"
}

"""
Code to parse output from CV and passes a dictionary that maps Piece letters to Poses to main()
Input: String goals
Output: Dictionary actual {Piece letters : PoseStamped Messages} that contain initial/actual poses for each piece
Assumptions: pieces are unique and objects_msg is formatted correctly
"""
def parseActual(objects_msg):
    colors = objects_msg.header.split()
    actual = {}
    for pose in objects_msg:
        piece = PoseStamped()
        piece.header.frame_id = "base"
        piece.pose = pose
        actual[colors[0]] = piece
        colors.pop(0)
    actual_subscriber.shutdown()
    main(actual)
    
actual_subscriber = rospy.Subscriber(OBJECTS_SUB_TOPIC, PoseArray, parseActual)

"""
Code to parse user input for goals and return a dictionary that maps Piece letters to Poses
Input: String goals
Returns: dictionary {Piece letters : PoseStamped Messages}
"""
def parseGoals(goals):
    goals = goals.split(".")
    for row_i in range(len(goals)):
        goals[row_i] = goals[row_i].split()
    # goals = [[None, "L", "L"], ["O","O", "L"], ["O","O", "L"]]
    goals = decipher_final_configuration(goals)
    return goals

"""
Main script to path plan and actuate using MoveItCommander
Input: Dictionary actual {Piece letters : PoseStamped Messages} that contain initial/actual poses for each piece
       Dictionary goals {Piece letters : PoseStamped Messages} that contain desired poses for each piece
"""
def actuate(actual, goals):
    planner = PathPlanner("right_arm")
    gripper = robot_gripper.Gripper('right')

    #Calibrate the gripper (other commands won't work unless you do this first)
    gripper.clear_calibration()
    gripper.calibrate()
    rospy.sleep(2.0)

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

    actual_list = actual.keys()

    raw_input("Press <Enter> to move the right arm to pick up a piece: ")
    while not rospy.is_shutdown() and len(actual_list) > 0:
        # Booleans to track state
        in_hand, picked, placed = False, False, False
        piece = actual_list.pop(0)

        ### PICKING UP PIECE FROM ACTUAL LOCATION ###
        while not rospy.is_shutdown() and not picked:
            try:
                original = actual[piece]
                # Pick up the piece in the original position
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
                    ## GRIPPER CLOSE (succ)
                    gripper.close()
                    rospy.sleep(2.0)
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
                    ## GRIPPER OPEN (release)
                    gripper.open()
                    rospy.sleep(2.0)
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

"""
Given actual, will prompt user for goals and pass in appropriate params to actuate
Input: Dictionary actual {Piece letters : PoseStamped Messages} that contain initial/actual poses for each piece
"""
def main(actual):

    # Get the initial position and orientation of each piece ###PARTH"S FUNCTION
    # x, y, z = 0.47, -0.85, 0.07
    x, y, z = 0.6, -0.4, -.1
    # x, y, z = .68,-0.85,-0.1
    
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
    actual = {"L" : original, "O": original, "Z": original, "S": original, "I": original}

    # Get the desired position and orientation of each piece by parsing the desired string
    # goals = raw_input("Enter in a 2D matrix of your desired layout, with each element separated by spaces and each row separated by periods: ")
    goals = "L O O G S S.L O O S S Z Z.L L I I I I Z Z"
    goals = parseGoals(goals)
    actuate(actual, goals)

    # # Loop to loop over goals
    # while goals:
    #     goals = parseGoals(goals)

    #     actuate(actual, goals)
        
    #     print("We have finished solving the puzzle. If you would like to continue: ")
    #     goals = None
    #     # goals = raw_input("Enter in a 2D matrix of your desired layout, with each element separated by spaces and each row separated by periods: ")

if __name__ == '__main__':
    main("")