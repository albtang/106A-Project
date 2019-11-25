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
    def block_I():
        # Determine orientation of the piece in the configuration passed in
        if j+3 < len(ideal[i]) and ideal[i][j+1] == "I" and ideal[i][j+2] == "I" and ideal[i][j+3] == "I": #horizontal
            ignore.append([i, j+1])
            ignore.append([i, j+2])
            ignore.append([i, j+3])
            #orientation
            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 1.0
            pose.pose.orientation.z = 0
            pose.pose.orientation.w = 0
        elif i+3 < len(ideal) and ideal[i+1][j] == "I" and ideal[i+2][j] == "I" and ideal[i+3][j] == "I": #vertical
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
    def block_Z():
        if i+1 < len(ideal) and j+2 < len(ideal[i+1]) and ideal[i][j+1] == "Z" and ideal[i+1][j+1] == "Z" and ideal[i+1][j+2] == "Z": #horizontal
            ignore.append([i, j+1])
            ignore.append([i+1, j+1])
            ignore.append([i+1, j+2])
            #orientation
            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 1.0
            pose.pose.orientation.z = 0
            pose.pose.orientation.w = 0
            print(ignore)
        elif i+2 < len(ideal) and j+1 < len(ideal[i])  and j+1 < len(ideal[i+1]) and ideal[i+1][j] == "Z" and ideal[i+1][j+1] == "Z" and ideal[i+2][j+1] == "Z": #horizontal: #vertical
            ignore.append([i+1, j])
            ignore.append([i+1, j+1])
            ignore.append([i+2, j+1])
            #orientation
            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 1.0
            pose.pose.orientation.z = 0
            pose.pose.orientation.w = 0
        pieces.get("Z").append(pose)
    def block_O():
        ignore.append([i, j+1])
        ignore.append([i+1, j])
        ignore.append([i+1, j+1])
        #orientation
        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 1.0
        pose.pose.orientation.z = 0
        pose.pose.orientation.w = 0
        pieces.get("O").append(pose)
    def block_L():
        if i+1 < len(ideal) and j+2 < len(ideal[i]) and ideal[i+1][j] == "L" and ideal[i][j+1] == "L" and ideal[i][j+2] == "L":
            ignore.append([i+1, j])
            ignore.append([i, j+1])
            ignore.append([i, j+2])
            #orientation
            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 1.0
            pose.pose.orientation.z = 0
            pose.pose.orientation.w = 0
        elif i+2 < len(ideal) and j+1 < len(ideal[i]) and j+1 < len(ideal[i+1]) and j+1 < len(ideal[i+2]) and ideal[i][j+1] == "L" and ideal[i+1][j+1] == "L" and ideal[i+2][j+1] == "L":
            ignore.append([i, j+1])
            ignore.append([i+1, j+1])
            ignore.append([i+2, j+1])
            #orientation
            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 1.0
            pose.pose.orientation.z = 0
            pose.pose.orientation.w = 0
        elif i+2 < len(ideal) and j+1 < len(ideal[i]) and j+1 < len(ideal[i+2]) and ideal[i+1][j] == "L" and ideal[i+2][j] == "L" and ideal[i+2][j+1] == "L":
            ignore.append([i+1, j])
            ignore.append([i+2, j])
            ignore.append([i+2, j+1])
            #orientation
            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 1.0
            pose.pose.orientation.z = 0
            pose.pose.orientation.w = 0
        elif i+1 < len(ideal) and j-2 >= 0 and ideal[i+1][j] == "L" and ideal[i+1][j-1] == "L" and ideal[i+1][j-2] == "L":
            ignore.append([i+1, j])
            ignore.append([i+1, j-1])
            ignore.append([i+1, j-2])
            #orientation
            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 1.0
            pose.pose.orientation.z = 0
            pose.pose.orientation.w = 0
        pieces.get("L").append(pose) 
    def block_J():  
        if i+1 < len(ideal) and j+2 < len(ideal[i+1]) and ideal[i+1][j] == "J" and ideal[i+1][j+1] == "J" and ideal[i+1][j+2] == "J":
            ignore.append([i+1, j])
            ignore.append([i+1, j+1])
            ignore.append([i+1, j+2])
            #orientation
            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 1.0
            pose.pose.orientation.z = 0
            pose.pose.orientation.w = 0
        elif i+2 < len(ideal) and j+1 < len(ideal[i]) and ideal[i][j+1] == "J" and ideal[i+1][j] == "J" and ideal[i+2][j] == "J":
            ignore.append([i, j+1])
            ignore.append([i+1, j])
            ignore.append([i+2, j])
            #orientation
            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 1.0
            pose.pose.orientation.z = 0
            pose.pose.orientation.w = 0
        elif i+2 < len(ideal) and j-1 >= 0 and ideal[i+1][j] == "J" and ideal[i+2][j] == "J" and ideal[i+2][j-1] == "J":
            ignore.append([i+1, j])
            ignore.append([i+2, j])
            ignore.append([i+2, j-1])
            #orientation
            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 1.0
            pose.pose.orientation.z = 0
            pose.pose.orientation.w = 0
        elif i+1 < len(ideal) and j+2 < len(ideal[i]) and j+2 < len(ideal[i+1]) and ideal[i][j+1] == "J" and ideal[i][j+2] == "J" and ideal[i+1][j+2] == "J":
            ignore.append([i, j+1])
            ignore.append([i, j+2])
            ignore.append([i+1, j+2])
            #orientation
            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 1.0
            pose.pose.orientation.z = 0
            pose.pose.orientation.w = 0
        pieces.get("J").append(pose)
    def block_S():
        if i+1 < len(ideal) and j+1 < len(ideal[i]) and j-1 >= 0 and ideal[i][j+1] == "S" and ideal[i+1][j] == "S" and ideal[i+1][j-1] == "S": #horizontal
            ignore.append([i, j+1])
            ignore.append([i+1, j])
            ignore.append([i+1, j-1])
            #orientation
            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 1.0
            pose.pose.orientation.z = 0
            pose.pose.orientation.w = 0
        elif i+2 < len(ideal) and j+1 < len(ideal[i+1]) and j+1 < len(ideal[i+2]) and ideal[i+1][j] == "S" and ideal[i+1][j+1] == "S" and ideal[i+2][j+1] == "S": #horizontal #vertical
            ignore.append([i+1, j])
            ignore.append([i+1, j+1])
            ignore.append([i+2, j+1])
            #orientation
            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 1.0
            pose.pose.orientation.z = 0
            pose.pose.orientation.w = 0
        pieces.get("S").append(pose)
    def block_T():
        if i+1 < len(ideal) and j+2 < len(ideal[i]) and j+1 < len(ideal[i+1]) and ideal[i][j+1] == "T" and ideal[i][j+2] == "T" and ideal[i+1][j+1] == "T":
            ignore.append([i, j+1])
            ignore.append([i, j+2])
            ignore.append([i+1, j+1])
            #orientation
            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 1.0
            pose.pose.orientation.z = 0
            pose.pose.orientation.w = 0
        elif i+2 < len(ideal) and j+1 < len(ideal[i+1]) and ideal[i+1][j] == "T" and ideal[i+1][j+1] == "T" and ideal[i+2][j] == "T":
            ignore.append([i+1, j])
            ignore.append([i+1, j+1])
            ignore.append([i+2, j])
            #orientation
            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 1.0
            pose.pose.orientation.z = 0
            pose.pose.orientation.w = 0
        elif i+2 < len(ideal) and j-1 >= 0 and ideal[i+1][j] == "T" and ideal[i+1][j-1] == "T" and ideal[i+2][j] == "T":
            ignore.append([i+1, j])
            ignore.append([i+1, j-1])
            ignore.append([i+2, j])
            #orientation
            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 1.0
            pose.pose.orientation.z = 0
            pose.pose.orientation.w = 0
        elif i+1 < len(ideal) and j+1 < len(ideal[i+1]) and j-1 >= 0 and ideal[i+1][j] == "T" and ideal[i+1][j+1] == "T" and ideal[i+1][j-1] == "T":
            ignore.append([i+1, j])
            ignore.append([i+1, j+1])
            ignore.append([i+1, j-1])
            #orientation
            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 1.0
            pose.pose.orientation.z = 0
            pose.pose.orientation.w = 0
        pieces.get("T").append(pose)
    
    pieces = {"I": [], "O": [], "J": [], "L": [], "S": [], "Z": [], "T": []}
    ignore = []
    block_size = 0.1016 # 4 inches in m
    # x, y, z = 0.47, -0.5, 0.07
    x, y, z = 0, 0, 0

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
                block_I()

            elif ideal[i][j] == "Z":
                block_Z()

            elif ideal[i][j] == "O" and ideal[i][j+1] == "O" and ideal[i+1][j] == "O" and ideal[i+1][j+1] == "O":
                block_O()

            elif ideal[i][j] == "L":
                block_L()

            elif ideal[i][j] == "J":
                block_J()

            elif ideal[i][j] == "S":
                block_S()

            elif ideal[i][j] == "T":
                block_T()

            else:
                continue
    return pieces