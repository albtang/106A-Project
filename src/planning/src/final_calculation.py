import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

'''
Function that translates a 2D Array specifying the final desired configuration to values in physical space
Authors: Parth Baokar, Jasmine Le, Albert Tang, Justin Villamor, Teresa Yang

Input: 2D Array, where elements correspond to blocks in space, labeled by letter to indicate what piece the block is a part of
Output: dictionary mapping letter to list of final desired Poses for each piece
'''

def decipher_final_configuration(ideal):
    pieces = {"I": [], "O": [], "J": [], "L": [], "S": [], "Z": [], "T": []}
    ignore = []
    block_size = 0.1016 # 4 inches in m
    x, y, z = 0.47, -0.5, 0.07

    for i in range(len(ideal)): #horizontal?
    	for j in range(len(ideal[i])): #vertical
            if [i,j] in ignore:
                ignore.remove([i,j])
                continue
            pose = PoseStamped()
            pose.header.frame_id = "base"
            # Determine relative position in relation to the other pieces
            pose.pose.position.x = x + i*block_size
            pose.pose.position.y = y + j*block_size
            pose.pose.position.z = z + 0

            # Determine the specific piece that the letter corresponds to
            if ideal[i][j] == "I":
                # Determine orientation of the piece in the configuration passed in
                if ideal[i,j+1] == "I" and ideal[i,j+2] == "I" and ideal[i,j+3] == "I": #horizontal
                    ignore.append([i, j+1])
                    ignore.append([i, j+2])
                    ignore.append([i, j+3])
                    #orientation
                    pose.pose.orientation.x = 0
                    pose.pose.orientation.y = 1.0
                    pose.pose.orientation.z = 0
                    pose.pose.orientation.w = 0
                else: #vertical
                    ignore.append([i+1, j])
                    ignore.append([i+2, j])
                    ignore.append([i+3, j])
                    #orientation
                    pose.pose.orientation.x = 0
                    pose.pose.orientation.y = 1.0
                    pose.pose.orientation.z = 0
                    pose.pose.orientation.w = 0
                # ADD POSITION AND ORIENTATION
                pieces.get("I").append(pose)

            elif ideal[i][j] == "Z":
                if ideal[i,j+1] == "Z": #horizontal
                    ignore.append([i+1, j])
                    ignore.append([i+1, j+1])
                    ignore.append([i+1, j+2])
                    #orientation
                    pose.pose.orientation.x = 0
                    pose.pose.orientation.y = 1.0
                    pose.pose.orientation.z = 0
                    pose.pose.orientation.w = 0
                else: #vertical
                    ignore.append([i, j+1])
                    ignore.append([i-1, j+1])
                    ignore.append([i-1, j+2])
                    #orientation
                    pose.pose.orientation.x = 0
                    pose.pose.orientation.y = 1.0
                    pose.pose.orientation.z = 0
                    pose.pose.orientation.w = 0
                pieces.get("Z").append(pose)

            elif ideal[i][j] == "O":
                ignore.append([i, j+1])
                ignore.append([i+1, j])
                ignore.append([i+1, j+1])
                #orientation
                pose.pose.orientation.x = 0
                pose.pose.orientation.y = 1.0
                pose.pose.orientation.z = 0
                pose.pose.orientation.w = 0
                pieces.get("O").append(pose)

            elif ideal[i][j] == "L":
                if ideal[i+1,j] == "L" and ideal[i, j+1] == "L":
                    ignore.append([i, j+1])
                    ignore.append([i+1, j])
                    ignore.append([i, j+2])
                    #orientation
                    pose.pose.orientation.x = 0
                    pose.pose.orientation.y = 1.0
                    pose.pose.orientation.z = 0
                    pose.pose.orientation.w = 0
                elif ideal[i+1,j] != "L" and ideal[i, j+1] == "L":
                    ignore.append([i, j+1])
                    ignore.append([i+1, j+1])
                    ignore.append([i+2, j+1])
                    #orientation
                    pose.pose.orientation.x = 0
                    pose.pose.orientation.y = 1.0
                    pose.pose.orientation.z = 0
                    pose.pose.orientation.w = 0
                elif ideal[i+1,j] == "L" and ideal[i, j+1] != "L":
                    ignore.append([i+1, j])
                    ignore.append([i+2, j])
                    ignore.append([i+2, j+1])
                    #orientation
                    pose.pose.orientation.x = 0
                    pose.pose.orientation.y = 1.0
                    pose.pose.orientation.z = 0
                    pose.pose.orientation.w = 0
                else:
                    ignore.append([i+1, j])
                    ignore.append([i+1, j-1])
                    ignore.append([i+1, j-2])
                    #orientation
                    pose.pose.orientation.x = 0
                    pose.pose.orientation.y = 1.0
                    pose.pose.orientation.z = 0
                    pose.pose.orientation.w = 0
                pieces.get("L").append(pose)   

            elif ideal[i][j] == "J":
                if ideal[i+1,j] == "J" and ideal[i+1, j+1] == "J":
                    ignore.append([i+1, j])
                    ignore.append([i+1, j+1])
                    ignore.append([i+1, j+2])
                    #orientation
                    pose.pose.orientation.x = 0
                    pose.pose.orientation.y = 1.0
                    pose.pose.orientation.z = 0
                    pose.pose.orientation.w = 0
                elif ideal[i+1,j] == "J" and ideal[i, j+1] == "J":
                    ignore.append([i, j+1])
                    ignore.append([i+1, j])
                    ignore.append([i+2, j])
                    #orientation
                    pose.pose.orientation.x = 0
                    pose.pose.orientation.y = 1.0
                    pose.pose.orientation.z = 0
                    pose.pose.orientation.w = 0
                elif ideal[i+1,j] == "J" and ideal[i+2, j] == "J":
                    ignore.append([i+1, j])
                    ignore.append([i+2, j])
                    ignore.append([i+2, j-1])
                    #orientation
                    pose.pose.orientation.x = 0
                    pose.pose.orientation.y = 1.0
                    pose.pose.orientation.z = 0
                    pose.pose.orientation.w = 0
                else:
                    ignore.append([i, j+1])
                    ignore.append([i, j+2])
                    ignore.append([i+1, j+2])
                    #orientation
                    pose.pose.orientation.x = 0
                    pose.pose.orientation.y = 1.0
                    pose.pose.orientation.z = 0
                    pose.pose.orientation.w = 0
                pieces.get("J").append(pose)

            elif ideal[i][j] == "S":
                if ideal[i,j+1] == "S": #horizontal
                    ignore.append([i, j+1])
                    ignore.append([i+1, j])
                    ignore.append([i+1, j-1])
                    #orientation
                    pose.pose.orientation.x = 0
                    pose.pose.orientation.y = 1.0
                    pose.pose.orientation.z = 0
                    pose.pose.orientation.w = 0
                else: #vertical
                    ignore.append([i+1, j])
                    ignore.append([i+1, j+1])
                    ignore.append([i+2, j+1])
                    #orientation
                    pose.pose.orientation.x = 0
                    pose.pose.orientation.y = 1.0
                    pose.pose.orientation.z = 0
                    pose.pose.orientation.w = 0
                pieces.get("S").append(pose)

            elif ideal[i][j] == "T":
                if ideal[i,j+1] == "T":
                    ignore.append([i, j+1])
                    ignore.append([i, j+2])
                    ignore.append([i+1, j+1])
                    #orientation
                    pose.pose.orientation.x = 0
                    pose.pose.orientation.y = 1.0
                    pose.pose.orientation.z = 0
                    pose.pose.orientation.w = 0
                elif ideal[i+2,j] == "T" and ideal[i+1, j+1] == "T":
                    ignore.append([i+1, j])
                    ignore.append([i+1, j+1])
                    ignore.append([i+2, j])
                    #orientation
                    pose.pose.orientation.x = 0
                    pose.pose.orientation.y = 1.0
                    pose.pose.orientation.z = 0
                    pose.pose.orientation.w = 0
                elif ideal[i+2,j] == "T" and ideal[i+1, j-1] == "T":
                    ignore.append([i+1, j])
                    ignore.append([i+1, j-1])
                    ignore.append([i+2, j])
                    #orientation
                    pose.pose.orientation.x = 0
                    pose.pose.orientation.y = 1.0
                    pose.pose.orientation.z = 0
                    pose.pose.orientation.w = 0
                else:
                    ignore.append([i+1, j])
                    ignore.append([i+1, j+1])
                    ignore.append([i+1, j-1])
                    #orientation
                    pose.pose.orientation.x = 0
                    pose.pose.orientation.y = 1.0
                    pose.pose.orientation.z = 0
                    pose.pose.orientation.w = 0
                pieces.get("T").append(pose)
                
            else:
                continue
    return pieces