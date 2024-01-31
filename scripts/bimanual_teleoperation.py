
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on July 27 2022
@author: Giovanni Franzese, Cognitive Robotics, TU Delft
Run this node in a terminal. 
Remember to run the spacenav launch file 
$ roslaunch spacenav_node classic.launch 
and to install the space nav before with 
$ sudo apt install spacenavd
$ sudo apt install ros-indigo-spacenav-node
"""

import rospy
import numpy as np
import quaternion # pip install numpy-quaternion
from quaternion import from_euler_angles
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray, Float32
from sensor_msgs.msg import Joy
from pynput.keyboard import Listener, KeyCode
from scipy.spatial.transform import Rotation

from SIMPLe_bimanual.utils import get_quaternion_from_euler

class BiManualTeleoperation():
    def __init__(self):
        self.control_freq=rospy.Rate(100)
        self.panda_right_offset = [0, 0, 0, 0, 0, 0]
        self.panda_left_offset = [0, 0, 0, 0, 0, 0]
        self.panda_right_curr_pos = None
        self.panda_left_curr_pos = None
        self.panda_right_joint_pos = None
        self.panda_left_joint_pos = None
        self.panda_right_joy_sub=rospy.Subscriber("/spacenav_right/joy", Joy, self.panda_right_spacenav_callback)
        self.panda_left_joy_sub=rospy.Subscriber("/spacenav_left/joy", Joy, self.panda_left_spacenav_callback)
        self.panda_right_pos_sub=rospy.Subscriber("/panda_dual/bimanual_cartesian_impedance_controller/panda_right_cartesian_pose", PoseStamped, self.panda_right_ee_pos_callback)
        self.panda_left_pos_sub=rospy.Subscriber("/panda_dual/bimanual_cartesian_impedance_controller/panda_left_cartesian_pose", PoseStamped, self.panda_left_ee_pos_callback)
        self.panda_right_goal_pub = rospy.Publisher('/panda_dual/bimanual_cartesian_impedance_controller/panda_right_equilibrium_pose', PoseStamped, queue_size=0)  
        self.panda_left_goal_pub = rospy.Publisher('/panda_dual/bimanual_cartesian_impedance_controller/panda_left_equilibrium_pose', PoseStamped, queue_size=0)  

    def panda_right_spacenav_callback(self, data):
        self.panda_right_offset[0]= 0.001*data.axes[0]
        self.panda_right_offset[1]= 0.001*data.axes[1]
        self.panda_right_offset[2]= 0.001*data.axes[2]
        self.panda_right_offset[3]= 0.01*data.axes[3]
        self.panda_right_offset[4]= 0.01*data.axes[4] 
        self.panda_right_offset[5]= 0.01*data.axes[5]


    def panda_left_spacenav_callback(self, data):
        self.panda_left_offset[0]= 0.001*data.axes[0]
        self.panda_left_offset[1]= 0.001*data.axes[1]
        self.panda_left_offset[2]= 0.001*data.axes[2]
        self.panda_left_offset[3]= 0.01*data.axes[3]
        self.panda_left_offset[4]= 0.01*data.axes[4] 
        self.panda_left_offset[5]= 0.01*data.axes[5]

    def panda_right_ee_pos_callback(self, data):
        self.panda_right_curr_pos = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
        self.panda_right_curr_ori = np.array([data.pose.orientation.w, data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z])

    def panda_left_ee_pos_callback(self, data):
        self.panda_left_curr_pos = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
        self.panda_left_curr_ori = np.array([data.pose.orientation.w, data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z])


  
    def control(self):

        panda_right_goal = PoseStamped()
        panda_right_goal.header.seq = 1
        panda_right_goal.header.stamp = rospy.Time.now()
        panda_right_goal.header.frame_id = "map"

        panda_right_goal.pose.position.x =  self.panda_right_curr_pos[0]
        panda_right_goal.pose.position.y =  self.panda_right_curr_pos[1]
        panda_right_goal.pose.position.z =  self.panda_right_curr_pos[2]
            #print("quat", quat) 
        panda_right_goal.pose.orientation.w = self.panda_right_curr_ori[0]    
        panda_right_goal.pose.orientation.x = self.panda_right_curr_ori[1] 
        panda_right_goal.pose.orientation.y = self.panda_right_curr_ori[2] 
        panda_right_goal.pose.orientation.z = self.panda_right_curr_ori[3] 
        self.panda_right_goal_pub.publish(panda_right_goal)  
        panda_right_quat_goal=np.quaternion(self.panda_right_curr_ori[0],self.panda_right_curr_ori[1],self.panda_right_curr_ori[2],self.panda_right_curr_ori[3])

        panda_left_goal = PoseStamped()
        panda_left_goal.header.seq = 1
        panda_left_goal.header.stamp = rospy.Time.now()
        panda_left_goal.header.frame_id = "map"

        panda_left_goal.pose.position.x =  self.panda_left_curr_pos[0]
        panda_left_goal.pose.position.y =  self.panda_left_curr_pos[1]
        panda_left_goal.pose.position.z =  self.panda_left_curr_pos[2]
            #print("quat", quat) 
        panda_left_goal.pose.orientation.w = self.panda_left_curr_ori[0]    
        panda_left_goal.pose.orientation.x = self.panda_left_curr_ori[1] 
        panda_left_goal.pose.orientation.y = self.panda_left_curr_ori[2] 
        panda_left_goal.pose.orientation.z = self.panda_left_curr_ori[3] 
        self.panda_left_goal_pub.publish(panda_left_goal)  
        panda_left_quat_goal=np.quaternion(self.panda_left_curr_ori[0],self.panda_left_curr_ori[1],self.panda_left_curr_ori[2],self.panda_left_curr_ori[3])

        while not rospy.is_shutdown(): 
   
            panda_right_goal.pose.position.x = panda_right_goal.pose.position.x + self.panda_right_offset[0]
            panda_right_goal.pose.position.y = panda_right_goal.pose.position.y + self.panda_right_offset[1]
            panda_right_goal.pose.position.z = panda_right_goal.pose.position.z + self.panda_right_offset[2]
            q_delta=get_quaternion_from_euler(self.panda_right_offset[3],self.panda_right_offset[4],self.panda_right_offset[5])
            panda_right_q_delta=np.quaternion(q_delta[0],q_delta[1],q_delta[2],q_delta[3])
            panda_right_quat_goal=panda_right_q_delta*panda_right_quat_goal

            panda_right_goal.pose.orientation.w = panda_right_quat_goal.w   
            panda_right_goal.pose.orientation.x = panda_right_quat_goal.x
            panda_right_goal.pose.orientation.y = panda_right_quat_goal.y
            panda_right_goal.pose.orientation.z = panda_right_quat_goal.z
        
                    

            panda_left_goal.pose.position.x = panda_left_goal.pose.position.x + self.panda_left_offset[0]
            panda_left_goal.pose.position.y = panda_left_goal.pose.position.y + self.panda_left_offset[1]
            panda_left_goal.pose.position.z = panda_left_goal.pose.position.z + self.panda_left_offset[2]
            q_delta=get_quaternion_from_euler(self.panda_left_offset[3],self.panda_left_offset[4],self.panda_left_offset[5])
            panda_left_q_delta=np.quaternion(q_delta[0],q_delta[1],q_delta[2],q_delta[3])
            panda_left_quat_goal=panda_left_q_delta*panda_left_quat_goal

            panda_left_goal.pose.orientation.w = panda_left_quat_goal.w   
            panda_left_goal.pose.orientation.x = panda_left_quat_goal.x
            panda_left_goal.pose.orientation.y = panda_left_quat_goal.y
            panda_left_goal.pose.orientation.z = panda_left_quat_goal.z
        
            self.panda_right_goal_pub.publish(panda_right_goal)    
            self.panda_left_goal_pub.publish(panda_left_goal)            
        
            self.control_freq.sleep()
        
    
#%%    
if __name__ == '__main__':
    rospy.init_node('Teleoperation', anonymous=True)

#%%
    teleoperation=BiManualTeleoperation()
    rospy.sleep(2)
    teleoperation.control()
