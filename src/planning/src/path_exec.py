#!/usr/bin/env python
"""
Path Planning Script for project
Author: Parth Baokar, Jasmine Le, Albert Tang, Justin Villamor, Teresa Yang, Valmik Prabhu
"""
import sys

from baxter_interface import Limb, gripper as robot_gripper
import rospy
import numpy as np
import traceback

from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import PoseStamped, PoseArray

from path_planner import PathPlanner
from final_calculation import decipher_final_configuration

### VARIABLES TO CHANGE FOR DEMO ###
CV = True
# CV = False
# PREDEFINED_GOAL = None
# PREDEFINED_GOAL = "J J J G G.Z Z J T G.I Z Z T T.I O O T G.I O O L G.I L L L G"
PREDEFINED_GOAL = "L O O J. L O O J. L L J J. Z Z G G. G Z Z G"
# ACTUAL = ["L", "J", "I", "O", "Z", "S", "T"]
ACTUAL_LETTERS = ["L"]
init_x, init_y, init_z = 0.5, .2, -.28
# x, y, z = 0.47, -0.85, -0.15
# x, y, z = 0.6, -0.4, -.15
# x, y, z = 0.47, .3, -.15
final_x, final_y, final_z = 0.8, -.4, -.28
# x, y, z = 1, -0.5, -.15
# x, y, z = -0.47, -0.5, -.15
# x, y, z = 0.4, -0.8, -.15
# x, y, z = 0.47, -0.85, -.15
# x, y, z = 0.47, -.4, -.15

### GLOBAL NAMES ###
NODE = 'moveit_node'
OBJECTS_SUB_TOPIC = "detected_objects"

"""
A mapping from color to piece letter
Used for processing actual configuration information from segmentation
TODO: add the rest of the colors when necessary
"""
ColorMapping = {
    "blue" : "O",
    "red" : "L",
    "purple" : "J",
    # "yellow" : "I",
    # "black" : "S",
    "green" : "Z"
    # "orange" : "T"
}

"""
Code to parse output from CV and passes a dictionary that maps Piece letters to Poses to main()
Input: String goals
Calls: main() with Dictionary actual {Piece letters : PoseStamped Messages} that contain initial/actual poses for each piece
Assumptions: pieces are unique and objects_msg is formatted correctly
"""

def parseActual(objects_msg):
    colors = objects_msg.header.frame_id.split()
    actual = {}
    for pose in objects_msg.poses:
        piece = PoseStamped()
        piece.header.frame_id = "base"
        piece.pose = pose
        piece.pose.orientation.x = 0.0
        piece.pose.orientation.y = -1.0
        piece.pose.orientation.z = 0.0
        piece.pose.orientation.w = 0.0
        actual[ColorMapping[colors[0]]] = piece
        colors.pop(0)
    main(actual)

"""
Code to parse user input for goals and return a dictionary that maps Piece letters to Poses
Input: String goals, float x, y, z for position of starting corner
Returns: dictionary {Piece letters : PoseStamped Messages}
"""
def parseGoals(goals, x, y, z):
    goals = goals.split(".")
    for row_i in range(len(goals)):
        goals[row_i] = goals[row_i].split()
    goals = decipher_final_configuration(goals, x, y, z)
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
    ##
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
    # orien_const = OrientationConstraint()
    # orien_const.link_name = "right_gripper";
    # orien_const.header.frame_id = "base";
    # orien_const.orientation.y = -1.0;
    # orien_const.absolute_x_axis_tolerance = 0.1;
    # orien_const.absolute_y_axis_tolerance = 0.1;
    # orien_const.absolute_z_axis_tolerance = 0.1;
    # orien_const.weight = 1.0;

    # List of letters corresponding to the pieces in the actual configuration
    actual_list = actual.keys()
    print(actual_list)
    
    # Creation of a copy PoseStamped message to use for z translation
    above = PoseStamped()
    above.header.frame_id = "base"

    raw_input("Press <Enter> to move the right arm to pick up a piece: ")
    while not rospy.is_shutdown() and len(actual_list) > 0:
        # Booleans to track state
        in_hand, picked, placed = False, False, False
        piece = actual_list[0]

        ### PICKING UP PIECE FROM ACTUAL LOCATION ###
        while not rospy.is_shutdown() and not picked:
            print("PICKING")
            try:
                original = actual[piece]
                print(original)

                # Setting above's value to correspond to just above original position
                above.pose.position.x = original.pose.position.x
                above.pose.position.y = original.pose.position.y
                above.pose.position.z = original.pose.position.z + 0.2
                above.pose.orientation = original.pose.orientation

                # Pick up the piece in the original position
                if not in_hand:
                    # Translate over such that the piece is above the actual position
                    plan = planner.plan_to_pose(above, [])
                    if not planner.execute_plan(plan):
                        raise Exception("Execution failed")

                    # Actually pick up the piece
                    plan = planner.plan_to_pose(original, [])
                    if not planner.execute_plan(plan):
                        raise Exception("Execution failed")
                    ## GRIPPER CLOSE (succ)
                    gripper.close()
                    rospy.sleep(2.0)
                    in_hand = True

                # Raise the arm a little bit so that other pieces are not affected
                plan = planner.plan_to_pose(above, [])
                if not planner.execute_plan(plan):
                    raise Exception("Execution failed")

                ### PLACING PIECE AT DESIRED LOCATION ###
                text = raw_input("Press <Enter> to move the right arm to place the piece or 'back' to redo the previous step: ")
                if text == "back":
                    picked = False
                else:
                    picked = True

            except Exception as e:
                print e
                traceback.print_exc()
            else:
                break

        while not rospy.is_shutdown() and picked and not placed:
            print("PLACING")

            try:
                goal = goals.get(piece)[0]
                print(goal)

                # Setting above's value to correspond to just above original position
                above.pose.position.x = goal.pose.position.x
                above.pose.position.y = goal.pose.position.y
                above.pose.position.z = goal.pose.position.z + 0.2
                above.pose.orientation = goal.pose.orientation

                if in_hand:
                    # Translate over such that the piece is above the desired position
                    plan = planner.plan_to_pose(above, [])
                    if not planner.execute_plan(plan):
                        raise Exception("Execution failed")

                    # Actually place the piece on table
                    plan = planner.plan_to_pose(goal, [])
                    if not planner.execute_plan(plan):
                        raise Exception("Execution failed")
                    ## GRIPPER OPEN (release)
                    gripper.open()
                    rospy.sleep(2.0)
                    in_hand = False

                # Raise the arm a little bit so that other pieces are not affected
                plan = planner.plan_to_pose(above, [])
                if not planner.execute_plan(plan):
                    raise Exception("Execution failed")
            
                text = raw_input("Press <Enter> to move onto the next piece or 'back': ")
                if text == "back":
                    placed = False
                    picked = True
                else:
                    placed = True
                    picked = False
                    goals.get(piece).pop(0)
                    actual_list.pop(0)

            except Exception as e:
                print e
                traceback.print_exc()
            else:
                break

"""
Given actual, will prompt user for goals and pass in appropriate params to actuate()
Input: Dictionary actual {Piece letters : PoseStamped Messages} that contain initial/actual poses for each piece
Calls: actuate()
"""
def main(actual):
    # If CV is False, pass in hard coded values
    if not CV:
        x, y, z = init_x, init_y, init_z

        # DUMMY POSE
        original = PoseStamped()
        original.header.frame_id = "base"
        original.pose.position.x = x
        original.pose.position.y = y
        original.pose.position.z = z
        original.pose.orientation.x = 0.0
        original.pose.orientation.y = -1.0
        original.pose.orientation.z = 0.0
        original.pose.orientation.w = 0.0

        actual = {}

        for letter in ACTUAL_LETTERS:
            actual[letter] = original

    # Get the desired position and orientation of each piece by parsing the desired string
    if PREDEFINED_GOAL is None:
        goals = raw_input("Enter in a 2D matrix of your desired layout, with each element separated by spaces and each row separated by periods: ")
    else:
        goals = PREDEFINED_GOAL

    x, y, z = final_x, final_y, final_z
    goals = parseGoals(goals, x, y, z)

    actuate(actual, goals)

if __name__ == '__main__':
    rospy.init_node(NODE)
    actual_subscriber = rospy.Subscriber(OBJECTS_SUB_TOPIC, PoseArray, parseActual)
    rospy.spin()
    main("")