#!/usr/bin/env python2.7
import scipy.version
import rospy
import actionlib
import numpy as np
import kinova_msgs.msg
import geometry_msgs.msg
import tf
import std_msgs.msg
import scipy
from kinova_msgs.srv import *
from sensor_msgs.msg import JointState
import std_msgs_stamped.msg
import argparse
import time , threading
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped as Pose_stamp
from std_msgs.msg import String as str_msg
import datetime
import time
import os
import math

rospy.init_node("CUS_logger")

last_action_start=0
last_action_end=0
acting=False
reading=False
menus=[["translation_relative","up","dowm","left","right"],
       ["rotation_relative","pan_up","pan_dowm","pan_left","pan_right"],
       ["forward_roll_relative","forward","backward","roll left","roll right"],
       ["forward_up_relative","forward","backward","up","down"],
       ["up_pivot_global","up","down","pivot_left","pivot_right"]]
boxid=-1
menuid=0
CURRENT_LABEL="none"
# declare lists
starttime=0
current_time=rospy.Time.now()

x_vel=     [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
y_vel=     [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
z_vel=     [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]

x_twist=   [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
y_twist=   [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
z_twist=   [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]

timestamps=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
tongue=    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
delay_dur=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    
last_dur=0
def tongue_callback(data):
    global last_action_start,last_action_end, acting,menuid,boxid,CURRENT_LABEL,reading
    strings=data.data
    global last_dur
    strings=strings.split('*')
    if(strings[0]=="none" and acting):
        timesteps=0
        timesum=0.0
        astamp=data.header.stamp
        last_action_end=astamp


        duration=(last_action_end-current_time).to_sec()

        print("pressing stopped"+str(duration)+"seconds ago")
        while(timesum-duration>0):
            timesum+=timestamps[len(timestamps)-1-timesteps]
            tongue[len(timestamps)-1-timesteps]=0
            timesteps+=1
        print("this was "+str(timesteps)+" ago")
        last_dur=duration
        threading.Timer(2,stopreading).start()


    acting=False
    print(strings[0])
    if(strings[0]=="act"):
        astamp=data.header.stamp
        last_action_start=astamp
        
        acting=True
        timesteps=0
        timesum=0.0

        duration=(last_action_start-current_time).to_sec()
        print("pressing started"+str(duration)+"seconds ago")
        while(timesum-duration>0):
            timesum+=timestamps[len(timestamps)-1-timesteps]
            tongue[len(timestamps)-1-timesteps]=1
            timesteps+=1
        reading=True
        print("this was "+str(timesteps)+" ago")
        last_dur=duration
    if(strings[0]=="contract"):
        last_action_start=data.header.stamp
        #acting=True
    if(strings[0]=="expand"):
        last_action_start=data.header.stamp
        #acting=True
    if(strings[0]=="left"):
        last_action_start=data.header.stamp
        #acting=True
    if(strings[0]=="right"):
        last_action_start=data.header.stamp
        #acting=True
    
    if(acting):
        CURRENT_LABEL="command:"+strings[0]+":<"+menus[menuid][0]+":"+menus[menuid][1+boxid]+">"
        print(CURRENT_LABEL)


def stopreading():

    global reading,timestamps,x_vel,x_twist,y_vel,y_twist,z_vel,z_twist,tongue, delay_dur
    reading=False
    
    file=0
    name='log_file_('+ CURRENT_LABEL+ ')'+str(time.time())+'.txt'
    file = open("./logs/"+name,"a")
    print(str(os.getcwd()))
    print("created file" +name)

    file.write("x_vel,y_vel.z_vel,x_twist.y,z_twist,tonguestate,since_activation \n")
    for x in range(len(timestamps)):
        line=str(timestamps[x])+","+str(x_vel[x])+","+str(y_vel[x])+","+str(z_vel[x])+","+str(x_twist[x])+","+str(y_twist[x])+","+str(z_twist[x])+","+str(tongue[x])+","+str(delay_dur[x])
        print("wrote line "+str(x))

        file.write(line+"\n")
    
    x_vel=     [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    y_vel=     [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    z_vel=     [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]

    x_twist=   [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    y_twist=   [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    z_twist=   [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]

    timestamps=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    tongue=    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    delay_dur=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    
def gaze_callback(data):
    global acting, boxid
    strin=data.data
    if(True):
        #update comand monitor chek
        
        if(strin=="box:1:"):
            boxid=0
        elif(strin=="box:2:"):
            boxid=1
        elif(strin=="box:3:"):
            boxid=2
        elif(strin=="box:4:"):
            boxid=3
    print("box "+str(boxid)+" from"+strin)
    


def menu_callback(data):
    global acting, menuid
    if(not acting):
        menuid=data.data
    print("got"+str(data.data))

def QuaternionNorm(Q_raw):
    qx_temp,qy_temp,qz_temp,qw_temp = Q_raw[0:4]
    qnorm = math.sqrt(qx_temp*qx_temp + qy_temp*qy_temp + qz_temp*qz_temp + qw_temp*qw_temp)
    qx_ = qx_temp/qnorm
    qy_ = qy_temp/qnorm
    qz_ = qz_temp/qnorm
    qw_ = qw_temp/qnorm
    Q_normed_ = [qx_, qy_, qz_, qw_]
    return Q_normed_


def Quaternion2EulerXYZ(Q_raw):
    Q_normed = QuaternionNorm(Q_raw)
    qx_ = Q_normed[0]
    qy_ = Q_normed[1]
    qz_ = Q_normed[2]
    qw_ = Q_normed[3]

    tx_ = math.atan2((2 * qw_ * qx_ - 2 * qy_ * qz_), (qw_ * qw_ - qx_ * qx_ - qy_ * qy_ + qz_ * qz_))
    ty_ = math.asin(2 * qw_ * qy_ + 2 * qx_ * qz_)
    tz_ = math.atan2((2 * qw_ * qz_ - 2 * qx_ * qy_), (qw_ * qw_ + qx_ * qx_ - qy_ * qy_ - qz_ * qz_))
    EulerXYZ_ = [tx_,ty_,tz_]
    return EulerXYZ_

lastlin=geometry_msgs.msg.Point()
last_ang=[0,0,0]


def robot_callback(data):
    global current_time, acting,reading, lastlin,last_ang
    global x_vel,x_twist,y_vel,y_twist,z_vel,z_twist,timestamps,tongue
    global delay_dur,last_dur
    
    
    header  = data.header
    stamp   = header.stamp
    pose   = data.pose
    linear_global  = pose.position
    linear = [linear_global.x-lastlin.x,linear_global.y-lastlin.y,linear_global.z-lastlin.z]
    lastlin=linear_global


    q= pose.orientation
    angular_global = Quaternion2EulerXYZ(QuaternionNorm([q.x,q.y,q.z,q.w]))
    angular=[angular_global[0]-last_ang[0],angular_global[1]-last_ang[1],angular_global[2]-last_ang[2]] 
    last_ang=angular_global

    if((not acting) and (not reading)):
        #scoot linear vectors
        x_vel=x_vel[1:len(x_vel)]
        x_vel.append(linear[0])
        y_vel=y_vel[1:len(y_vel)]
        y_vel.append(linear[1])
        z_vel=z_vel[1:len(z_vel)]
        z_vel.append(linear[2])

        #scoot angular vectors
        x_twist=x_twist[1:len(x_twist)]
        x_twist.append(angular[0])
        y_twist=y_twist[1:len(y_twist)]
        y_twist.append(angular[1])
        z_twist=z_twist[1:len(z_twist)]
        z_twist.append(angular[2])
        #print("arrayis :"+str(len(z_twist)))
        #print("menus"+str(menuid)+  str(boxid))

        #update current time
        lasttime=current_time
        current_time=stamp
        # scoot current time and add deltatime
        timestamps=timestamps[1:len(timestamps)]
        timestamps.append((current_time-lasttime).to_sec())
        
    else:
        

        delay_dur.append(last_dur)
        if(last_dur!=0):
            last_dur=0


        if(acting):
            tongue.append(1)
        else:
            tongue.append(0)
        x_vel.append(linear[0])
        y_vel.append(linear[1])
        z_vel.append(linear[2])
        x_twist.append(angular[0])
        y_twist.append(angular[1])
        z_twist.append(angular[2])
        #update current time
        lasttime=current_time
        current_time=stamp
        # scoot current time and add deltatime
        timestamps.append((current_time-lasttime).to_sec())
        #print(CURRENT_LABEL)

        



sub_IT=0
sub_UI=0


sub_TRANS=0
if __name__ == '__main__':
    
    sub_IT=rospy.Subscriber("/itongue_comands",std_msgs_stamped.msg.StringStamped  , tongue_callback)
    sub_UI=rospy.Subscriber("/gaze_comands",str_msg , gaze_callback)
    sub_TRANS=rospy.Subscriber("/j2n6s300_driver/out/tool_pose",Pose_stamp , robot_callback)
    sub_men=rospy.Subscriber("/driver/menu/selected",std_msgs.msg.Int16,menu_callback)
    topic_name = '/' + 'j2n6s300_' + 'driver/in/cartesian_velocity'
	#publish joint torque commands
    pub_vel_mover = rospy.Publisher("/j2n6s300_driver/in/cartesian_velocity", kinova_msgs.msg.PoseVelocity, queue_size=10)
    rospy.spin()
        





    print("stopped_spinning")
