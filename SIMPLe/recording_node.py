#!/usr/bin/env python

from unicodedata import name
import rospy
import numpy as np
from datetime import datetime
import pathlib
from geometry_msgs.msg import PoseStamped, WrenchStamped
from std_msgs.msg import Bool
from pynput.keyboard import Listener, KeyCode
class Recorder:
    def __init__(self):
        self.recording_freq = 20
        self.recording = False

        rospy.init_node("recording_node")

        rospy.Subscriber("/cartesian_pose",
                         PoseStamped, self.panda_pose_callback)
        
        
        
        rospy.Subscriber("/equilibrium_pose", PoseStamped,self.panda_attractor_callback)
        
       

        rospy.Subscriber("/force_torque_ext", WrenchStamped, self.panda_force_torque_callback)
       

        

        rospy.Subscriber("/recording", Bool, self.recording_callback)

        self.listener = Listener(on_press=self._on_press)
        self.listener.start()

        self.cart_pos = np.zeros(3)
        self.cart_ori = np.zeros(4)



        self.panda_attractor_pos = np.zeros(3)

        self.panda_attractor_t = np.zeros(2)

        self.panda_attractor_ori= np.zeros(4)


        self.panda_force_torque = np.zeros(6)

      

        self.equilibrium_distance_pose = np.zeros(3)



    def recording_callback(self, data):
        self.recording=data

    def panda_force_torque_callback(self,data):
        self.panda_force_torque = np.array([data.wrench.force.x, data.wrench.force.y, data.wrench.force.z, data.wrench.torque.x, data.wrench.torque.y, data.wrench.torque.z])

    
    


    def panda_pose_callback(self, data):
        self.cart_pos = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
        self.cart_ori = np.array([data.pose.orientation.w, data.pose.orientation.x, data.pose.orientation.y,
                        data.pose.orientation.z])


    def panda_attractor_callback(self,data):
        self.panda_attractor_pos = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
        self.panda_attractor_ori = np.array([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])
        self.panda_attractor_t = np.array([data.header.stamp.secs, data.header.stamp.nsecs])


    def _on_press(self, key):
        # This function runs on the background and checks if a keyboard key was pressed
        if key == KeyCode.from_char('e'):
            self.recording = False
            rospy.set_param('/recording', False)
        if key == KeyCode.from_char('m'):
            self.recording = True
            rospy.set_param('/recording', True)

    def record(self):
        print("Press m to start recording, press e to stop")
        r = rospy.Rate(self.recording_freq)
        timestamp = datetime.now().strftime("%d_%H_%M_%S")


        self.recorded_cart_pose = np.r_[self.cart_pos, self.cart_ori]
        self.recorded_attractor_pose = np.r_[self.panda_attractor_pos, self.panda_attractor_ori]
        self.recorded_attractor_time = np.r_[self.panda_attractor_t]
        self.recorded_force_torque = np.r_[self.panda_force_torque]

        while not self.recording:
            pass

        print('Recording')
        while self.recording:
            # print("Test")

            self.recorded_cart_pose = np.c_[self.recorded_cart_pose, np.r_[self.cart_pos, self.cart_ori]]
            self.recorded_attractor_pose = np.c_[self.recorded_attractor_pose,np.r_[self.panda_attractor_pos, self.panda_attractor_ori]]
            self.recorded_attractor_time = np.c_[self.recorded_attractor_time, np.r_[self.panda_attractor_t]]
            self.recorded_force_torque = np.c_[self.recorded_force_torque, np.r_[self.panda_force_torque]]
            
            r.sleep()

        self.recording = False
        self.save(filename=str(timestamp))


    def save(self, filename = 'last'):
        np.savez(str(pathlib.Path().resolve()) + '/results/cleaning/' + str(filename) + '.npz',
            recorded_cart_pose = self.recorded_cart_pose,
            recorded_attractor_pose = self.recorded_attractor_pose,
            recorded_force_torque = self.recorded_force_torque,
            )

    def load(self, filename):
        data = np.load(str(pathlib.Path().resolve()) + '/results/cleaning/' + str(filename) + '.npz')
        return data





