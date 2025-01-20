#!/usr/bin/env python2.7
import scipy.version
import rospy
import actionlib
import numpy as np
import kinova_msgs.msg
import geometry_msgs.msg
import tf
import std_msgs.msg
import math
import thread
import scipy
from scipy.spatial.transform import Rotation
from kinova_msgs.srv import *
from sensor_msgs.msg import JointState
import argparse
import std_msgs_stamped.msg
from geometry_msgs.msg import PoseStamped as Pose_stamp
from std_msgs.msg import String as str_msg

actuate=False

MAT_hand_base=1
ROT_hand_base=1
MAT_base_hand=1
ROT_base_hand=1
ROT_base_hand=np.eye(4)

CURRENT_TOUNGE_COMMAND="none"
def QuaternionNorm(Q_raw):
    qx_temp,qy_temp,qz_temp,qw_temp = Q_raw[0:4]
    qnorm = math.sqrt(qx_temp*qx_temp + qy_temp*qy_temp + qz_temp*qz_temp + qw_temp*qw_temp)
    qx_ = qx_temp/qnorm
    qy_ = qy_temp/qnorm
    qz_ = qz_temp/qnorm
    qw_ = qw_temp/qnorm
    Q_normed_ = [qx_, qy_, qz_, qw_]
    return Q_normed_

def quaternion_rotation_matrix(Q):
    """
    Covert a quaternion into a full three-dimensional rotation matrix.
 
    Input
    :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
 
    Output
    :return: A 3x3 element matrix representing the full 3D rotation matrix. 
             This rotation matrix converts a point in the local reference 
             frame to a point in the global reference frame.
    """
    # Extract the values from Q
    q0 = Q[0]
    q1 = Q[1]
    q2 = Q[2]
    q3 = Q[3]
     
    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
     
    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
     
    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1
     
    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])
                            
    return rot_matrix

fingerpos=100

def gripper_client(finger_positions, prefix):
    """Send a gripper goal to the action server."""
    print("1")
    action_address = '/' + prefix + '_driver/fingers_action/finger_positions'

    client = actionlib.SimpleActionClient(action_address,
                                          kinova_msgs.msg.SetFingersPositionAction)
    client.wait_for_server()
    print("2")
    goal = kinova_msgs.msg.SetFingersPositionGoal()
    goal.fingers.finger1 = float(finger_positions[0])
    goal.fingers.finger2 = float(finger_positions[1])
    goal.fingers.finger3 = float(finger_positions[2])
    client.send_goal(goal)
    print("3")
    if client.wait_for_result(rospy.Duration(50.0)):
        print("4")
        return client.get_result()
    else:
        print("e")
        client.cancel_all_goals()
        rospy.logwarn('        the gripper action timed-out')
        return None
# 
def itounge_callback(msg): # when a button activates or deactivates on the tongue interface
    global CURRENT_TOUNGE_COMMAND
    CURRENT_TOUNGE_COMMAND=msg.data.split(';')[0]
    global acting,expand,contract,menu_up,menu_down

    print(CURRENT_TOUNGE_COMMAND)
    prepared=False
    str_com=CURRENT_TOUNGE_COMMAND

    acting=expand=contract=menu_down=menu_up=False

    if(str_com=="act"): # if the button is pressed initiate the action, so that the main while loop will know to execute it on its next loop
        print("acting")
        acting=True      
    elif(str_com=="contract"):
        contract=True
    elif(str_com=="expand"):
        expand=True
    elif(str_com=="left"):
        menu_up=True
    elif(str_com=="right"):
        menu_down=True
    elif(str_com=="none"):
        print("unpressed")
    # this intermediary boolean structure is made due to the fact that we otherwise would be unable to register an unpress as 
    # calbacks cannot interupt previous callbacks

UI_DIR=[0,0,0,0,0,0] # the screenspace direction the gaze comand
gaze_active=False # describes wether or not what the gaze tracker is curently looking at an activation surface
vel=[0,0,0]
def gaze_ui_callback(msg):# when the pupil tracker node changes the curently selected box update the corosponding direction
    strin=msg.data
    global UI_DIR,gaze_active
    global move_modes,move_current
    global vel

    print(strin)
    vel=[0,0,0]# instantiate the velocity 
    # the gaze active is used to keep track of weather or not we are curently looking at an active box
    # this is used to validate weather or not we should act after pressing the act button,
    # it also means if we go from an inactive box to an active box the movement will actuate instantly if the act button on the tongue interface is pressed
    if(strin=="box:0:"): # if the cursor is outside all active boxes
        vel=[0,0,0] 
        gaze_active=False
    elif(strin=="box:1:"): # if the top box is active
        vel=[0,-1,0]
        gaze_active=True #
    elif(strin=="box:2:"): # if the bottom box is active
        gaze_active=True
        vel=[0,1,0]
    elif(strin=="box:3:"): # if the left box is active
        gaze_active=True
        vel=[1,0,0]
    elif(strin=="box:4:"): # if the right box is active
        gaze_active=True
        vel=[-1,0,0]

def get_comand_vector_based_on_menu(vel):
    #this command is called when the robot is acting according to the current menu, and the command seleted by the eyegaze
    # load the information from the robots current position, and the current selected menu
    global ROT_base_hand,ROT_hand_base,move_mode
   
    #establish default commands 
    command=[0,0,0,0,0,0]
    command_joint=[0,0,0,0,0,0]

    # if the robot is in translation mode
    if(move_mode=="translation_camera"):
        #it will move acording to the axies perpendicular to the current view direction
        # flip the directionss perpendicular to the current view directuin
        dir_Flip=np.array([vel[0],-vel[1],vel[2],1])
        vec=np.matmul(ROT_base_hand,dir_Flip )
        command=[vec[0],vec[1],vec[2],0,0,0] #load the command into the command vector  
    # if the robot is in rotation mode
    elif(move_mode=="rotation_camera"):
        #it will move in a panning otion along the horisontal or vertical axies based on the curent view direction
        dir_Flip=[vel[1],vel[0],vel[2],1]
        vec=np.matmul(ROT_base_hand,dir_Flip)
        lis=vec
        # the twist vector is already in local cordinates in order to avoid singularities while calculating
        command=[0,0,0,dir_Flip[0],dir_Flip[1],dir_Flip[2]]
    elif(move_mode=="forward_roll"):
        #we would like to move along the cameras forward axis, this is the -z direction
        dir_Flip=np.array([0,0,-vel[1],1])
        vec=np.matmul(ROT_base_hand,dir_Flip)# then rotate the 
        command=[vec[0],vec[1],vec[2],0,0,vel[0]]
    
    elif(move_mode=="forward_vertical"):
        # we would like to move in the vertical plane that is coincident with the cameras forward axies
        dir_Flip=np.array([vel[2],-vel[0],-vel[1],1])
        vec=np.matmul(ROT_base_hand,dir_Flip )
        command=[vec[0],vec[1],vec[2],0,0,0] 

    elif(move_mode=="basepivot_handraise"):
        # we would like to move the first joint while moving side to side, to avoid having to do multiple menu switches during a sinle turning motion
        dir_Flip=np.array([vel[0],vel[1],vel[2],1])
        #vec=np.matmul(ROT_base_hand,dir_Flip )
        command=[0,0,-dir_Flip[1],0,0,0]
        command_joint=[-dir_Flip[0]*9,0,0,0,0,0]

    return command,command_joint

def robot_transform_callback(msg): # when the robot publishes the pose of its end effector
    #pose contains postion xyz and orientation quaternion
    x=msg.pose.position.x
    y=msg.pose.position.y
    z=msg.pose.position.z

    Q=msg.pose.orientation 

    global ROT_base_hand,ROT_hand_base
    qn=QuaternionNorm([Q.x,Q.y,Q.z,Q.w]) # normalise quaternion such that it can be made into a rotation matrix
    ROT_base_hand=tf.transformations.quaternion_matrix(qn)# make the quaternion into a quaternion 
    #this is saved in a global variable, and later used to calculate the command vectors




# basic command framework, for sending cartesian velocity commands to the robot
def move_cart_com(position, orientation, prefix):
    """Send a cartesian goal to the action server."""
    action_address = '/' + prefix + 'driver/pose_action/tool_pose'
    client = actionlib.SimpleActionClient(action_address, kinova_msgs.msg.ArmPoseAction)
    client.wait_for_server()

    goal = kinova_msgs.msg.ArmPoseGoal()
    goal.pose.header = std_msgs.msg.Header(frame_id=(prefix + 'link_base'))
    goal.pose.pose.position = geometry_msgs.msg.Point(
        x=position[0], y=position[1], z=position[2])
    goal.pose.pose.orientation = geometry_msgs.msg.Quaternion(
        x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3])

    print('goal.pose in client 1: {}'.format(goal.pose.pose)) # debug

    client.send_goal(goal)

    if client.wait_for_result(rospy.Duration(200.0)):
        return client.get_result()
    else:
        client.cancel_all_goals()
        print('        the cartesian action timed-out')
        return None
    
speed=1
def updatespeed(msg):# callback from the itounge containing the "speed" information
    global speed
    #print(msg.data)
    speed=msg.data
    #speed=1

def getspeed(): # retrives the global variable "speed" even if the code is in a while loop
    global speed
    #print(speed)
    return speed

# variables containing information about different movenode logic in order to make shure the curent selected menu is persistent
move_modes=["translation_camera","rotation_camera","forward_roll","forward_vertical","basepivot_handraise"]
move_mode="translation_camera"
move_current=0
max_moves=len(move_modes)

#global ros publishers and subscribers
sub_IT=1
sub_UI=1
sub_TRANS=1
pub_vel_mover=1
pub_joint_vel_mover=1
pub_cart_mover=1
PUB_CUR_MENU=1
SUB_SPEED_HAND=1

def get_bools():
    # retrives global variables, signifying which commands are currently active
    global acting,expand,contract,menu_up,menu_down
    return[acting,contract,expand,menu_up,menu_down]

def aprox_list(list_A,list_B):
    ret=True
    for x in range(len(list_A)):
        if(np.abs(list_A[x]-list_B[x])>0.01):
            ret=False
    return ret



acting=False
expand=False
contract=False
menu_up=False
menu_down=False



if __name__ == '__main__':
    # initate all the current node and the topic subscribers and publishers
    rospy.init_node("CUS_driver")
    sub_IT=rospy.Subscriber("itongue_comands",  std_msgs_stamped.msg.StringStamped , itounge_callback)
    sub_UI=rospy.Subscriber("gaze_comands",str_msg , gaze_ui_callback)
    sub_TRANS=rospy.Subscriber("/j2n6s300_driver/out/tool_pose",Pose_stamp , robot_transform_callback)
    topic_name = '/' + 'j2n6s300_' + 'driver/in/cartesian_velocity'
	#publish joint torque commands
    pub_vel_mover = rospy.Publisher("/j2n6s300_driver/in/cartesian_velocity", kinova_msgs.msg.PoseVelocity, queue_size=1)
    pub_joint_vel_mover = rospy.Publisher("/j2n6s300_driver/in/joint_velocity", kinova_msgs.msg.JointVelocity, queue_size=1)
    SUB_SPEED_HAND= rospy.Subscriber("itongue_speed",std_msgs.msg.Float32,updatespeed)
    PUB_CUR_MENU=rospy.Publisher("/driver/menu/selected",std_msgs.msg.Int16,queue_size=1)
    rate=rospy.Rate(100)
    rate2=rospy.Rate(10)
    # run the while loop until the ros node stops
    while( not rospy.is_shutdown()):
        bools=get_bools()
        acting1=bools[0]
        contract1=bools[1]
        expand1=bools[2]
        menu_up1=bools[3]
        menu_down1=bools[4]
        rate.sleep()

        if(acting1):
            print("acting")
            if(gaze_active):
                print("1")
                #
                cartVel,jointVel = get_comand_vector_based_on_menu(vel) # get the movement direction based 
                poseVelCmd = kinova_msgs.msg.PoseVelocity()
                jointVelCmd = kinova_msgs.msg.JointVelocity()
                CMD_SPEED=getspeed()
                trans_speed=0.1*CMD_SPEED # set the liner speed multiplier
                angular_speed=1*CMD_SPEED # set the angular speed multiplier
                #print(str(cartVel)+"cartvel")
                #print(str(jointVel)+"jointvel")
                rate3 = rospy.Rate(100) # at 100 hz
                if(not aprox_list(cartVel,[0,0,0,0,0,0])):
                    
                    
                    while (acting):# until the act button is unpressed 
                        CMD_SPEED=getspeed()
                        #CMD_SPEED=1
                        #print("3"+str(CMD_SPEED))
                        trans_speed=0.1*CMD_SPEED # set the liner speed multiplier
                        angular_speed=10*CMD_SPEED # set the angular speed multiplier
                        # build movement message for cartesian movement

                        poseVelCmd.twist_linear_x = cartVel[0] * trans_speed;
                        poseVelCmd.twist_linear_y = cartVel[1] * trans_speed;
                        poseVelCmd.twist_linear_z = cartVel[2] * trans_speed;
                        poseVelCmd.twist_angular_x = cartVel[3]* angular_speed;
                        poseVelCmd.twist_angular_y = cartVel[4]* angular_speed;
                        poseVelCmd.twist_angular_z = cartVel[5]* angular_speed;
                        pub_vel_mover.publish(poseVelCmd) # keep sending the command
                        rate3.sleep() # wait for 0.01 second
                    print("4")
                elif(not aprox_list(jointVel, [0,0,0,0,0,0])):
                  
                    
                    while (acting):
                        CMD_SPEED=getspeed()
                        trans_speed=0.1*CMD_SPEED # set the liner speed multiplier
                        angular_speed=1*CMD_SPEED # set the angular speed multiplier
                        # build movement message for cartesian movement
                        jointVelCmd.joint1 = jointVel[0] * angular_speed;
                        jointVelCmd.joint2 = jointVel[1] * angular_speed;
                        jointVelCmd.joint3 = jointVel[2] * angular_speed;
                        jointVelCmd.joint4 = jointVel[3] * angular_speed;
                        jointVelCmd.joint5 = jointVel[4] * angular_speed;
                        jointVelCmd.joint6 = jointVel[5] * angular_speed;
                        pub_joint_vel_mover.publish(jointVelCmd)
                        rate3.sleep()
                        #sprint("7")
                    print("8")

                
        if contract1 :
            print("beginning close")
            while(contract):# open fingers
                fingerpos=fingerpos-1500 # decrease finger value
                if(fingerpos<0):# do not go below zero
                    fingerpos=0
                gripper_client([fingerpos,fingerpos,fingerpos],"j2n6s300") # send the finger postion to the gripper client
                print("closing"+str(fingerpos))
                rate2.sleep()
            print("stop_closing")
        if expand1 :
            print("beginning open")
            while(expand): # close fingers
                fingerpos=fingerpos+1500 #increase finger value
                if(fingerpos>6800): # do not go above max
                    fingerpos=6800
                gripper_client([fingerpos,fingerpos,fingerpos],"j2n6s300")
                print("opening"+str(fingerpos))
                rate2.sleep()
            print("stop_opening")

        if(menu_down1):# change the menu, increase the current selected value
            move_current=np.abs((max_moves+move_current+1)%max_moves)
            PUB_CUR_MENU.publish(int(move_current))
            move_mode=move_modes[move_current]
            print(move_mode)
            menu_down=False
        if(menu_up1): # change the menu, decrease the curet selected value
            move_current=np.abs((max_moves+move_current-1)%max_moves)
            PUB_CUR_MENU.publish(int(move_current))
            move_mode=move_modes[move_current]
            print(move_mode)
            menu_up=False
        

        
    

    print("stopped_spinning")
