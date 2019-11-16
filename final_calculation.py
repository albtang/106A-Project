from geometry_msgs import Pose
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

    for i in ideal: #horizontal?
    	for j in i: #vertical
            if [i,j] in ignore:
                ignore.remove([i,j])
                continue
            pose = Pose()
            # Determine relative position in relation to the other pieces
            pose.position.x = i*block_size
            pose.position.y = j*block_size
            pose.position.z = 0

            # Determine the specific piece that the letter corresponds to
            if ideal[i,j] == "I":
                # Determine orientation of the piece in the configuration passed in
    			if ideal[i,j+1] == "I" and ideal[i,j+2] == "I" and ideal[i,j+3] == "I": #horizontal
    				ignore.append([i, j+1])
                    ignore.append([i, j+2])
                    ignore.append([i, j+3])
                    #orientation
                    pose.orientation.x = 0
                    pose.orientation.y = 0
                    pose.orientation.z = 0
                    pose.orientation.w = 1.0
                else: #vertical
                    ignore.append([i+1, j])
                    ignore.append([i+2, j])
                    ignore.append([i+3, j])
                    #orientation
                    pose.orientation.x = 0
                    pose.orientation.y = 0
                    pose.orientation.z = 0
                    pose.orientation.w = 1.0
                # ADD POSITION AND ORIENTATION
                pieces["I"] = pieces.get("I").add(pose)

    		elif ideal[i,j] == "Z":
                if ideal[i,j+1] == "Z": #horizontal
                    ignore.append([i+1, j])
                    ignore.append([i+1, j+1])
                    ignore.append([i+1, j+2])
                    #orientation
                    pose.orientation.x = 0
                    pose.orientation.y = 0
                    pose.orientation.z = 0
                    pose.orientation.w = 1.0
                else: #vertical
                    ignore.append([i, j+1])
                    ignore.append([i-1, j+1])
                    ignore.append([i-1, j+2])
                    #orientation
                    pose.orientation.x = 0
                    pose.orientation.y = 0
                    pose.orientation.z = 0
                    pose.orientation.w = 1.0
                pieces["Z"] = pieces.get("Z").add(pose)

    		elif ideal[i,j] == "O":
                ignore.append([i, j+1])
                ignore.append([i+1, j])
                ignore.append([i+1, j+1])
                #orientation
                pose.orientation.x = 0
                pose.orientation.y = 0
                pose.orientation.z = 0
                pose.orientation.w = 1.0
                pieces["O"] = pieces.get("O").add(pose)

    		elif ideal[i,j] == "L":
                if ideal[i+1,j] == "L" and ideal[i, j+1] == "L":
                    ignore.append([i, j+1])
                    ignore.append([i+1, j])
                    ignore.append([i, j+2])
                    #orientation
                    pose.orientation.x = 0
                    pose.orientation.y = 0
                    pose.orientation.z = 0
                    pose.orientation.w = 1.0
                elif ideal[i+1,j] != "L" and ideal[i, j+1] == "L":
                    ignore.append([i, j+1])
                    ignore.append([i+1, j+1])
                    ignore.append([i+2, j+1])
                    #orientation
                    pose.orientation.x = 0
                    pose.orientation.y = 0
                    pose.orientation.z = 0
                    pose.orientation.w = 1.0
                elif ideal[i+1,j] == "L" and ideal[i, j+1] != "L":
                    ignore.append([i+1, j])
                    ignore.append([i+2, j])
                    ignore.append([i+2, j+1])
                    #orientation
                    pose.orientation.x = 0
                    pose.orientation.y = 0
                    pose.orientation.z = 0
                    pose.orientation.w = 1.0
                else:
                    ignore.append([i+1, j])
                    ignore.append([i+1, j-1])
                    ignore.append([i+1, j-2])
                    #orientation
                    pose.orientation.x = 0
                    pose.orientation.y = 0
                    pose.orientation.z = 0
                    pose.orientation.w = 1.0
                pieces["L"] = pieces.get("L").add(pose)   

    		elif ideal[i,j] == "J":
                if ideal[i+1,j] == "J" and ideal[i+1, j+1] == "J":
                    ignore.append([i+1, j])
                    ignore.append([i+1, j+1])
                    ignore.append([i+1, j+2])
                    #orientation
                    pose.orientation.x = 0
                    pose.orientation.y = 0
                    pose.orientation.z = 0
                    pose.orientation.w = 1.0
                elif ideal[i+1,j] == "J" and ideal[i, j+1] == "J":
                    ignore.append([i, j+1])
                    ignore.append([i+1, j])
                    ignore.append([i+2, j])
                    #orientation
                    pose.orientation.x = 0
                    pose.orientation.y = 0
                    pose.orientation.z = 0
                    pose.orientation.w = 1.0
                elif ideal[i+1,j] == "J" and ideal[i+2, j] == "J":
                    ignore.append([i+1, j])
                    ignore.append([i+2, j])
                    ignore.append([i+2, j-1])
                    #orientation
                    pose.orientation.x = 0
                    pose.orientation.y = 0
                    pose.orientation.z = 0
                    pose.orientation.w = 1.0
                else:
                    ignore.append([i, j+1])
                    ignore.append([i, j+2])
                    ignore.append([i+1, j+2])
                    #orientation
                    pose.orientation.x = 0
                    pose.orientation.y = 0
                    pose.orientation.z = 0
                    pose.orientation.w = 1.0
                pieces["J"] = pieces.get("J").add(pose)

    		elif ideal[i,j] == "S":
                if ideal[i,j+1] == "S": #horizontal
                    ignore.append([i, j+1])
                    ignore.append([i+1, j])
                    ignore.append([i+1, j-1])
                    #orientation
                    pose.orientation.x = 0
                    pose.orientation.y = 0
                    pose.orientation.z = 0
                    pose.orientation.w = 1.0
                else: #vertical
                    ignore.append([i+1, j])
                    ignore.append([i+1, j+1])
                    ignore.append([i+2, j+1])
                    #orientation
                    pose.orientation.x = 0
                    pose.orientation.y = 0
                    pose.orientation.z = 0
                    pose.orientation.w = 1.0
                pieces["S"] = pieces.get("S").add(pose)

    		elif ideal[i,j] == "T":
                if ideal[i,j+1] == "T":
                    ignore.append([i, j+1])
                    ignore.append([i, j+2])
                    ignore.append([i+1, j+1])
                    #orientation
                    pose.orientation.x = 0
                    pose.orientation.y = 0
                    pose.orientation.z = 0
                    pose.orientation.w = 1.0
                elif ideal[i+2,j] == "T" and ideal[i+1, j+1] == "T":
                    ignore.append([i+1, j])
                    ignore.append([i+1, j+1])
                    ignore.append([i+2, j])
                    #orientation
                    pose.orientation.x = 0
                    pose.orientation.y = 0
                    pose.orientation.z = 0
                    pose.orientation.w = 1.0
                elif ideal[i+2,j] == "T" and ideal[i+1, j-1] == "T":
                    ignore.append([i+1, j])
                    ignore.append([i+1, j-1])
                    ignore.append([i+2, j])
                    #orientation
                    pose.orientation.x = 0
                    pose.orientation.y = 0
                    pose.orientation.z = 0
                    pose.orientation.w = 1.0
                else:
                    ignore.append([i+1, j])
                    ignore.append([i+1, j+1])
                    ignore.append([i+1, j-1])
                    #orientation
                    pose.orientation.x = 0
                    pose.orientation.y = 0
                    pose.orientation.z = 0
                    pose.orientation.w = 1.0
                pieces["T"] = pieces.get("T").add(pose)
                
    		else:
                continue