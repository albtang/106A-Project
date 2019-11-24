#!/usr/bin/env python
# -*- coding: iso-8859-1 -*-
import sys
import cProfile, pstats
import time 
import rospy
import roslib; roslib.load_manifest("moveit_python")
import moveit_commander
import moveit_msgs.msg
import baxter_interface
import geometry_msgs.msg
from moveit_python import PlanningSceneInterface, MoveGroupInterface
from geometry_msgs.msg import PoseStamped, PoseArray
from sensor_msgs.msg import Range

# Define initial parameters.
rospy.init_node('pnp', anonymous=True)
# Initialize the move_group API.
moveit_commander.roscpp_initialize(sys.argv)
# Connect the arms to the move group.
both_arms = moveit_commander.MoveGroupCommander('both_arms')
right_arm = moveit_commander.MoveGroupCommander('right_arm')
left_arm = moveit_commander.MoveGroupCommander('left_arm')
# Allow replanning to increase the odds of a solution.
right_arm.allow_replanning(True)
left_arm.allow_replanning(True)
# Set the arms reference frames.
right_arm.set_pose_reference_frame('base')
left_arm.set_pose_reference_frame('base')
# Create baxter_interface limb instance.
leftarm = baxter_interface.limb.Limb('left')
rightarm = baxter_interface.limb.Limb('right')
# Initialize the planning scene interface.
p = PlanningSceneInterface("base")
# Create baxter_interface gripper instance.
leftgripper = baxter_interface.Gripper('left')
rightgripper = baxter_interface.Gripper('right')
leftgripper.calibrate()
rightgripper.calibrate()
leftgripper.open()
rightgripper.open()

def set_current_position(arm, *arg):
    # Function to add the current position as the first point for a movement.
    if(arm=='left'):
        current_position=left_arm.get_current_pose()
    if(arm=='right'):
        current_position=right_arm.get_current_pose()

    current_pos = geometry_msgs.msg.Pose()
    current_pos.position.x = current_position.pose.position.x
    current_pos.position.y = current_position.pose.position.y
    current_pos.position.z = current_position.pose.position.z
    current_pos.orientation.x = current_position.pose.orientation.x
    current_pos.orientation.y = current_position.pose.orientation.y
    current_pos.orientation.z = current_position.pose.orientation.z
    current_pos.orientation.w = current_position.pose.orientation.w
    i=len(arg)
    if(i==1):
	waypoints=arg[0]
        waypoints.append(current_pos)
    return current_pos

def move(arm, *arg):
    # The cartesian path will be interpolated at a resolution of 0.1 cm
    # which is why the eef_step in cartesian translation is specify as 0.001. 
    # The jump threshold is specify as 0.0, effectively disabled.
    # This function is limited to 3 points but more can be added.
    fraction = 0
    attempts=0
    state=0
    waypoints = []
    set_current_position(arm, waypoints)
    # i is the number of waypoints.
    i=len(arg)
    waypoints.append(arg[0])
    # "goal" is the endposition of the movement, if there are more points then it will contain the last one.
    goal=arg[0]
    goal_x=goal.position.x
    goal_y=goal.position.y
    goal_z=goal.position.z
    if(i>1):
        goal=arg[1]
        goal_x=goal.position.x
        goal_y=goal.position.y
        goal_z=goal.position.z
        waypoints.append(arg[1])
    if(i>2):
        goal=arg[2]
        goal_x=goal.position.x
        goal_y=goal.position.y
        goal_z=goal.position.z
        waypoints.append(arg[2])

    if(arm=='right'):
        right_arm.set_start_state_to_current_state()
        # This function computes a cartesian path for the waypoints. It calculates points with a
        # maximum step size of 1 mm between the waypoints. It return the plan and the fraction
        # which says how good it followed the requested trajectory.
        # (example: fraction= 0.95.454545 -> followed 95.454545% of requested trajectory)
        (plan, fraction) = right_arm.compute_cartesian_path (waypoints, 0.001, 0.0, True)
        right_arm.execute(plan, wait=True) 
        # Read the position of the right arm to compare it with the goal.
        a=right_arm.get_current_pose()
	x_pos= a.pose.position.x
	y_pos= a.pose.position.y
	z_pos= a.pose.position.z
        # Waiting up to 3 seconds that the goal position is reached. (If it fail state=1)
	# It is also required to check that the movement is finished because it continues directly
        # after the command right_arm.execute() with the next code lines.
        while not((abs(z_pos-goal_z)< 0.01) and (abs(y_pos-goal_y)< 0.01) and (abs(x_pos-goal_x)< 0.01)):
            a=right_arm.get_current_pose()
	    x_pos= a.pose.position.x
	    y_pos= a.pose.position.y
	    z_pos= a.pose.position.z        
            time.sleep(0.5)
            if(attempts>6):
                print("----->cartesian path failed!<-----")
                state=1
            attempts +=1
        time.sleep(1)
        return state       

    if(arm=='left'):
        left_arm.set_start_state_to_current_state()
        (plan, fraction) = left_arm.compute_cartesian_path (waypoints, 0.001, 0.0, True)
        left_arm.execute(plan, wait=True)
        # Read the position of the left arm to compare it with the goal.
        a=left_arm.get_current_pose()
	x_pos= a.pose.position.x
	y_pos= a.pose.position.y
	z_pos= a.pose.position.z

        while not((abs(z_pos-goal_z)< 0.01) and (abs(y_pos-goal_y)< 0.01) and (abs(x_pos-goal_x)< 0.01)):
            a=left_arm.get_current_pose()
	    x_pos= a.pose.position.x
	    y_pos= a.pose.position.y
	    z_pos= a.pose.position.z
            time.sleep(0.5)
            if(attempts>6):
                print("----->cartesian path failed!<-----") 
                state=1      
            attempts +=1
        time.sleep(1)
        return state       

def measure_table():
    # This function measure the size and the position of the table. Add the value for offset_zero_point.
    offset_zero_point=0.903
    # Define positions.
    pos1 = {'left_e0': -1.69483279891317, 'left_e1':  1.8669726956453, 'left_s0': 0.472137005716569, 'left_s1': -0.38852045702393034, 'left_w0': -1.9770933862776057, 'left_w1': -1.5701993084642143, 'left_w2': -0.6339059781326424, 'right_e0': 1.7238109084167481, 'right_e1': 1.7169079948791506, 'right_s0': 0.36930587426147465, 'right_s1': -0.33249033539428713, 'right_w0': -1.2160632682067871, 'right_w1': 1.668587600115967, 'right_w2': -1.810097327636719}
    lpos1 = {'left_e0': -1.69483279891317, 'left_e1':  1.8669726956453, 'left_s0': 0.472137005716569, 'left_s1': -0.38852045702393034, 'left_w0': -1.9770933862776057, 'left_w1': -1.5701993084642143, 'left_w2': -0.6339059781326424}
    rpos1 = {'right_e0': 1.7238109084167481, 'right_e1': 1.7169079948791506, 'right_s0': 0.36930587426147465, 'right_s1': -0.33249033539428713, 'right_w0': -1.2160632682067871, 'right_w1': 1.668587600115967, 'right_w2': -1.810097327636719}

    m_z_start = geometry_msgs.msg.Pose()
    m_z_start.position.x = 0.55
    m_z_start.position.y = 0
    m_z_start.position.z = 0.0
    m_z_start.orientation.x = 1.0
    m_z_start.orientation.y = 0.0
    m_z_start.orientation.z = 0.0
    m_z_start.orientation.w = 0.0

    m_x_start = geometry_msgs.msg.Pose()
    m_x_start.position.x = 0.824555206312
    m_x_start.position.y = -0.101147200174
    m_x_start.position.z = 0
    m_x_start.orientation.x = 0.999078899126
    m_x_start.orientation.y = -0.0370175672297
    m_x_start.orientation.z = 0.0134534998151
    m_x_start.orientation.w = 0.0170310416472

    # For the right arm.
    m_y_start_1 = geometry_msgs.msg.Pose()
    m_y_start_1.position.x = 0.502944492492
    m_y_start_1.position.y = -0.759184953936
    m_y_start_1.position.z = 0
    m_y_start_1.orientation.x = 0.695094141364
    m_y_start_1.orientation.y = -0.718718693339
    m_y_start_1.orientation.z = 0.00984382718873
    m_y_start_1.orientation.w = 0.0138084595126
    # For the left arm.
    m_y_start_2 = geometry_msgs.msg.Pose()
    m_y_start_2.position.x = 0.569637271212
    m_y_start_2.position.y = 0.86081004269
    m_y_start_2.position.z = 0
    m_y_start_2.orientation.x = -0.752397350016
    m_y_start_2.orientation.y = -0.657989479311
    m_y_start_2.orientation.z = -0.0240898615418
    m_y_start_2.orientation.w = 0.0191768447787

    # The start values for the attached table in MoveIt.
    table_size_x = 1.5
    table_size_y = 2
    table_size_z = 0.8
    center_x = 0
    center_y = 0
    center_z = 0

    both_arms.set_joint_value_target(pos1)
    both_arms.plan()
    both_arms.go(wait=True)
    # cProfile to measure the performance (time) of the task.
    pr = cProfile.Profile()
    pr.enable()
    # Start to measure the height of the table.
    state=move("right", m_z_start)
    right_ir_sensor =rospy.wait_for_message("/robot/range/right_hand_range/state", Range)
    # At first move with 10 cm steps and if the range is smaller then 25 cm with 1 cm steps.
    while(right_ir_sensor.range> 0.25):
        if not state:
            m_z_start.position.z -=0.1
        state=move("right", m_z_start)
        right_ir_sensor =rospy.wait_for_message("/robot/range/right_hand_range/state", Range) 
        print "-----> The distance to table:",right_ir_sensor.range,"m"
    while(right_ir_sensor.range> 0.12):
        if not state:
            m_z_start.position.z -=0.01
        state=move("right", m_z_start)
        right_ir_sensor =rospy.wait_for_message("/robot/range/right_hand_range/state", Range) 
        print "-----> The distance to table:",right_ir_sensor.range,"m"
    time.sleep(3)
    right_ir_sensor =rospy.wait_for_message("/robot/range/right_hand_range/state", Range) 
    distance=right_ir_sensor.range
    print "-----> The distance to table:", distance,"m"
    a=right_arm.get_current_pose()
    z_pos= a.pose.position.z
    # From the ground to the zero point in MoveIt with the base frame are 0.903 m.
    # The z position of the tip of the gripper and the distance between
    # the tip and the table must be substract.
    # (zpos is negative for table < 90 cm)
    # The value 0.099 is the distance from the ir sensor to the gripper tip.
    table_size_z = offset_zero_point+z_pos-(distance-0.099)
    print "-----> The table size in z direction is:", table_size_z," m"
    center_z = -offset_zero_point+table_size_z/2
    print "-----> The center of z is the position:", center_z,"m"
    # Initialize the position z for the next measurements.
    m_y_start_1.position.z = -offset_zero_point+table_size_z+0.025
    m_y_start_2.position.z = -offset_zero_point+table_size_z+0.025
    m_x_start.position.z = -offset_zero_point+table_size_z+0.025
    # Add table as attached object.
    p.attachBox('table', table_size_x, table_size_y, table_size_z, center_x, center_y, center_z, 'base', touch_links=['pedestal'])
    p.waitForSync()
    # Move to the start position.
    both_arms.set_joint_value_target(pos1)
    both_arms.plan()
    both_arms.go(wait=True)
    # Move to the right and left border point.
    right_arm.set_pose_target(m_y_start_1)
    right_arm.plan()
    right_arm.go(wait=True)
    left_arm.set_pose_target(m_y_start_2)
    left_arm.plan()
    left_arm.go(wait=True)
    # Start to measure the size and position in y direction of the table. At first y1 endpoint.
    right_ir_sensor =rospy.wait_for_message("/robot/range/right_hand_range/state", Range)
    distance = right_ir_sensor.range
    # If the table is at the border point under the gripper the default value -0.8 is used.
    if distance < 0.15:
        y1_endpoint= -0.8
    else:
        y1_endpoint= 0
    while(right_ir_sensor.range> 0.2 and m_y_start_1.position.y < 0.2):
        m_y_start_1.position.y +=0.1
        move("right", m_y_start_1)
        right_ir_sensor =rospy.wait_for_message("/robot/range/right_hand_range/state", Range) 
    while right_ir_sensor.range< 0.2 and y1_endpoint == 0:
        m_y_start_1.position.y -= 0.01
        move("right", m_y_start_1)
        right_ir_sensor =rospy.wait_for_message("/robot/range/right_hand_range/state", Range) 
    time.sleep(3)
    if y1_endpoint == 0: 
        a=right_arm.get_current_pose()
	y_pos= a.pose.position.y
        # The ir sensor is not in the middle of the hand and a small safety distance is necessary.
        y1_endpoint= y_pos-0.04
    print "-----> y1 endpoint:",y1_endpoint,"m"
    # Start to measure the second y2 endpoint.
    left_ir_sensor =rospy.wait_for_message("/robot/range/left_hand_range/state", Range)
    distance = left_ir_sensor.range
    print "-----> The distance to table:", distance,"m"
    # If the table is at the border point under the gripper the default value 1.2 is used.
    if distance < 0.15:
        y2_endpoint= 1.2
    else:
        y2_endpoint= 0
    while(left_ir_sensor.range> 0.2 and m_y_start_2.position.y > 0.2):
        m_y_start_2.position.y -=0.1
        move("left", m_y_start_2)
        left_ir_sensor =rospy.wait_for_message("/robot/range/left_hand_range/state", Range) 
    while left_ir_sensor.range< 0.2 and y2_endpoint == 0:
        m_y_start_2.position.y += 0.01
        move("left", m_y_start_2)
        left_ir_sensor =rospy.wait_for_message("/robot/range/left_hand_range/state", Range) 
    time.sleep(3)
    if y2_endpoint == 0: 
        a=left_arm.get_current_pose()
	y_pos= a.pose.position.y
        # The ir sensor is not in the middle of the hand and a small safety distance is necessary.
        y2_endpoint= y_pos+0.04
        print "-----> y2 endpoint:",y2_endpoint,"m"
    # The y1 endpoint must be negative and the y2 endpoint positive.
    table_size_y = abs(y1_endpoint)+abs(y2_endpoint)
    print "-----> The table size in y direction is:", table_size_y,"m"
    center_y = y1_endpoint+ table_size_y/2-0.04
    print "-----> The center of y is the position:", center_y,"m"
    # Remove the old table and add one with the new value.
    p.removeAttachedObject('table')
    p.attachBox('table', table_size_x, table_size_y, table_size_z, center_x, center_y, center_z, 'base', touch_links=['pedestal'])
    p.waitForSync()
    # Move to the start position.
    both_arms.set_joint_value_target(pos1)
    both_arms.plan()
    both_arms.go(wait=True)
    # Move to the border point in the x direction.
    right_arm.set_pose_target(m_x_start)
    right_arm.plan()
    right_arm.go(wait=True)

    right_ir_sensor =rospy.wait_for_message("/robot/range/right_hand_range/state", Range)
    distance = right_ir_sensor.range
    # The x endpoint in the direction of the robot is allway 0.1.
    x1_endpoint= 0.1
    # If the table is at the border point under the gripper the default value 1.1 is used.
    if distance < 0.15:
        x2_endpoint= 1.1
    else:
        x2_endpoint= 0
    while(right_ir_sensor.range> 0.2 and m_x_start.position.x > 0.2):
        m_x_start.position.x -=0.1
        move("right", m_x_start)
        right_ir_sensor =rospy.wait_for_message("/robot/range/right_hand_range/state", Range) 
    while right_ir_sensor.range< 0.2 and x2_endpoint == 0:
        m_x_start.position.x += 0.01
        move("right", m_x_start)
        right_ir_sensor =rospy.wait_for_message("/robot/range/right_hand_range/state", Range) 
    time.sleep(3)
    if x2_endpoint == 0: 
        a=right_arm.get_current_pose()
	x_pos= a.pose.position.x
        # The ir sensor is not in the middle of the hand and a small safety distance is necessary.
        x2_endpoint= x_pos+0.04
        print "-----> x2 endpoint:", x2_endpoint,"m"
    table_size_x = abs(x2_endpoint)-abs(x1_endpoint)
    print "-----> The table size in x direction is:", table_size_x,"m"
    center_x = x1_endpoint+ table_size_x/2
    print "-----> The center of x is the position:", center_x,"m"
    # Remove the old table and add one with the new value.
    p.removeAttachedObject('table')
    # Add table as attached object. It is most possible to approach up to approximately 0.5 cm.
    # (Point on the table: -0.175)
    # Size of the box in x-, y- and z-dimension.
    # The x, y, z positions in link_name frame ('base') where the center of the box is.
    # The touch_links are robot links that can touch this object.
    # The value can be determined with right_arm.get_current_pose(). 
    # The pose value of z must be in every space direct of the table the
    # same height otherwise the pedestal must be configured.  
    # The zero point from MoveIt (with right_arm.get_current_pose())
    # and the function from the baxter_interface 
    # (rightarm.endpoint_pose()) is different.
    # The distance from the zero point from the link_name frame 'base' to 
    # the ground is for MoveIt 0.903 m.
    p.attachBox('table', table_size_x, table_size_y, table_size_z, center_x, center_y, center_z, 'base', touch_links=['pedestal'])
    p.waitForSync()
    # Move to the start position.
    right_arm.set_joint_value_target(rpos1)
    right_arm.plan()
    right_arm.go(wait=True)

    pr.disable()
    sortby = 'cumulative'
    ps=pstats.Stats(pr).sort_stats(sortby).print_stats(0.0)
    print "-----> The table size in x direction is:", table_size_x,"m"
    print "-----> The table size in y direction is:", table_size_y,"m"
    print "-----> The table size in z direction is:", table_size_z,"m"
    print "-----> The center of x is the position:", center_x,"m"
    print "-----> The center of y is the position:", center_y,"m"
    print "-----> The center of z is the position:", center_z,"m"
    time.sleep(100)
    moveit_commander.roscpp_shutdown()
    # Exit MoveIt.
    moveit_commander.os._exit(0)
if __name__=='__main__':
    try:
        rospy.init_node('pnp', anonymous=True)
        measure_table()
    except rospy.ROSInterruptException:
        pass
