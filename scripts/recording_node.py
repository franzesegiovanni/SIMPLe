#!/usr/bin/env python

from unicodedata import name
import rospy
import numpy as np
from datetime import datetime
import pathlib
from sensor_msgs.msg import JointState, Joy
from geometry_msgs.msg import PoseStamped, Vector3, WrenchStamped
from std_msgs.msg import Float32MultiArray, Empty
from pynput.keyboard import Listener, KeyCode

from apriltag_ros.msg import AprilTagDetectionArray

class Recorder:
    def __init__(self):
        self.recording_freq = 20
        self.recording = True

        rospy.init_node("recording_node")

        rospy.Subscriber("/panda_dual/bimanual_cartesian_impedance_controller/panda_left_cartesian_pose",
                         PoseStamped, self.panda_left_pose_callback)
        rospy.Subscriber("/panda_dual/bimanual_cartesian_impedance_controller/panda_right_cartesian_pose",
                         PoseStamped, self.panda_right_pose_callback)

        rospy.Subscriber("/panda_left/stiffness", Float32MultiArray, self.panda_left_stiffness_callback)
        rospy.Subscriber("/panda_right/stiffness", Float32MultiArray, self.panda_right_stiffness_callback)

        rospy.Subscriber("/panda_dual/bimanual_cartesian_impedance_controller/panda_left_equilibrium_pose", PoseStamped,self.panda_left_attractor_callback)
        rospy.Subscriber("/panda_dual/bimanual_cartesian_impedance_controller/panda_right_equilibrium_pose", PoseStamped,self.panda_right_attractor_callback)

        rospy.Subscriber("/spacenav_left/joy", Joy, self.teleop_callback_left)
        rospy.Subscriber("/spacenav_right/joy", Joy, self.teleop_callback_right)

        rospy.Subscriber("/force_torque_right_ext", WrenchStamped, self.panda_right_force_torque_callback)
        rospy.Subscriber("/force_torque_left_ext", WrenchStamped, self.panda_left_force_torque_callback)

        rospy.Subscriber("/panda_dual/bimanual_cartesian_impedance_controller/equilibrium_distance", PoseStamped, self.equilibrium_distance_callback)

        rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.box_location_callback)


        rospy.set_param("/dual_teaching/recording", False)

        self.listener = Listener(on_press=self._on_press)
        self.listener.start()

        self.panda_right_cart_pos = np.zeros(3)
        self.panda_right_cart_ori = np.zeros(4)

        self.panda_left_cart_pos = np.zeros(3)
        self.panda_left_cart_ori = np.zeros(4)

        self.panda_right_attractor_pos = np.zeros(3)
        self.panda_left_attractor_pos = np.zeros(3)

        self.panda_right_attractor_t = np.zeros(2)
        self.panda_left_attractor_t = np.zeros(2)

        self.panda_right_attractor_ori= np.zeros(4)
        self.panda_left_attractor_ori= np.zeros(4)

        self.panda_right_feedback = np.zeros(3)
        self.panda_left_feedback = np.zeros(3)

        self.panda_right_stiffness = np.zeros(7)
        self.panda_left_stiffness = np.zeros(7)

        self.panda_right_force_torque = np.zeros(6)
        self.panda_left_force_torque = np.zeros(6)

        self.execution_factor = 1

        self.equilibrium_distance_pose = np.zeros(3)

        self.box_location_position=np.zeros(3)
        self.box_location_orientation=np.zeros(4)

    def box_location_callback(self, msg):
        self.box_location_position=np.array([msg.detections[0].pose.pose.pose.position.x, msg.detections[0].pose.pose.pose.position.y, msg.detections[0].pose.pose.pose.position.z]) 
        self.box_location_orientation=np.array([msg.detections[0].pose.pose.pose.orientation.w, msg.detections[0].pose.pose.pose.orientation.x,msg.detections[0].pose.pose.pose.orientation.y,msg.detections[0].pose.pose.pose.orientation.z])
        #print(self.box_location_position)
        #print(self.box_location_orientation)
    def panda_right_force_torque_callback(self,data):
        self.panda_right_force_torque = np.array([data.wrench.force.x, data.wrench.force.y, data.wrench.force.z, data.wrench.torque.x, data.wrench.torque.y, data.wrench.torque.z])

    def panda_left_force_torque_callback(self,data):
        self.panda_left_force_torque = np.array([data.wrench.force.x, data.wrench.force.y, data.wrench.force.z, data.wrench.torque.x, data.wrench.torque.y, data.wrench.torque.z])

    def panda_left_pose_callback(self, data):
        self.panda_left_cart_pos = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
        self.panda_left_cart_ori = np.array([data.pose.orientation.w, data.pose.orientation.x, data.pose.orientation.y,
                        data.pose.orientation.z])

    def panda_right_pose_callback(self, data):
        self.panda_right_cart_pos = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
        self.panda_right_cart_ori = np.array([data.pose.orientation.w, data.pose.orientation.x, data.pose.orientation.y,
                        data.pose.orientation.z])

    def panda_left_stiffness_callback(self, stiffness):
        self.panda_left_stiffness = np.array([stiffness.data[0],stiffness.data[1],stiffness.data[2],stiffness.data[3],stiffness.data[4],stiffness.data[5],stiffness.data[6]])

    def panda_right_stiffness_callback(self, stiffness):
        self.panda_right_stiffness = np.array([stiffness.data[0],stiffness.data[1],stiffness.data[2],stiffness.data[3],stiffness.data[4],stiffness.data[5],stiffness.data[6]])

    def panda_left_attractor_callback(self,data):
        self.panda_left_attractor_pos = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
        self.panda_left_attractor_ori = np.array([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])
        self.panda_left_attractor_t = np.array([data.header.stamp.secs, data.header.stamp.nsecs])

    def panda_right_attractor_callback(self,data):
        self.panda_right_attractor_pos = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
        self.panda_right_attractor_ori = np.array([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])
        self.panda_right_attractor_t = np.array([data.header.stamp.secs, data.header.stamp.nsecs])

    def teleop_callback_left(self, data):
        self.panda_left_feedback = np.array(data.axes)

    def teleop_callback_right(self, data):
        self.panda_right_feedback = np.array(data.axes)

    def equilibrium_distance_callback(self, data):
        self.equilibrium_distance_pose = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
        self.equilibrium_distance_t = np.array([data.header.stamp.secs, data.header.stamp.nsecs])

    def get_params(self):
        self.execution_factor = rospy.get_param("/dual_teaching/execution_factor")

    def _on_press(self, key):
        # This function runs on the background and checks if a keyboard key was pressed
        if key == KeyCode.from_char('n'):
            self.recording = False
            rospy.set_param('/dual_teaching/recording', False)
        if key == KeyCode.from_char('m'):
            self.recording = True
            rospy.set_param('/dual_teaching/recording', True)

    def record(self):
        print("Press m to start recording, press n to stop")
        r = rospy.Rate(self.recording_freq)
        timestamp = datetime.now().strftime("%d_%H_%M_%S")
        self.recording = rospy.get_param('/dual_teaching/recording')

        while not self.recording:
            # print("Test")
            self.recording = rospy.get_param('/dual_teaching/recording')
            r.sleep()

        self.get_params()

        self.recorded_cart_pose = np.r_[self.panda_right_cart_pos, self.panda_right_cart_ori,self.panda_left_cart_pos, self.panda_left_cart_ori]
        self.recorded_attractor_pose = np.r_[self.panda_right_attractor_pos, self.panda_right_attractor_ori, self.panda_left_attractor_pos, self.panda_left_attractor_ori]
        self.recorded_attractor_time = np.r_[self.panda_right_attractor_t, self.panda_left_attractor_t]
        self.recorded_feedback = np.r_[self.panda_right_feedback, self.panda_left_feedback]
        self.recorded_equilibrium_distance = self.equilibrium_distance_pose
        self.recorded_execution_factor = self.execution_factor
        self.recorded_stiffness = np.r_[self.panda_right_stiffness, self.panda_left_stiffness]
        self.recorded_force_torque = np.r_[self.panda_right_force_torque, self.panda_left_force_torque]
        self.recorded_tag_position= self.box_location_position
        self.recorded_tag_orientation = self.box_location_orientation
        print('Recording')

        while self.recording:
            # print("Test")
            self.recording = rospy.get_param('/dual_teaching/recording')

            self.get_params()

            self.recorded_cart_pose = np.c_[self.recorded_cart_pose, np.r_[self.panda_right_cart_pos, self.panda_right_cart_ori,self.panda_left_cart_pos, self.panda_left_cart_ori]]
            self.recorded_attractor_pose = np.c_[self.recorded_attractor_pose,np.r_[self.panda_right_attractor_pos, self.panda_right_attractor_ori, self.panda_left_attractor_pos, self.panda_left_attractor_ori]]
            self.recorded_attractor_time = np.c_[self.recorded_attractor_time, np.r_[self.panda_right_attractor_t, self.panda_left_attractor_t]]
            self.recorded_feedback = np.c_[self.recorded_feedback,np.r_[self.panda_right_feedback, self.panda_left_feedback]]
            self.recorded_equilibrium_distance = np.c_[self.recorded_equilibrium_distance,self.equilibrium_distance_pose]
            self.recorded_execution_factor = np.c_[self.recorded_execution_factor,self.execution_factor]
            self.recorded_stiffness = np.c_[self.recorded_stiffness,np.r_[self.panda_right_stiffness, self.panda_left_stiffness]]
            self.recorded_force_torque = np.c_[self.recorded_force_torque, np.r_[self.panda_right_force_torque, self.panda_left_force_torque]]
            self.recorded_tag_position= np.c_[self.recorded_tag_position,self.box_location_position ]
            self.recorded_tag_orientation = np.c_[self.recorded_tag_orientation, self.box_location_orientation]
            r.sleep()

        self.recording = False
        self.save(filename=str(timestamp))


    def save(self, filename = 'last'):
        np.savez(str(pathlib.Path().resolve()) + '/recordings/' + str(filename) + '.npz',
            recorded_cart_pose = self.recorded_cart_pose,
            recorded_attractor_pose = self.recorded_attractor_pose,
            recorded_feedback = self.recorded_feedback,
            recorded_equilibrium_distance = self.recorded_equilibrium_distance,
            recorded_execution_factor = self.recorded_execution_factor,
            recorded_stiffness = self.recorded_stiffness,
            recorded_force_torque = self.recorded_force_torque,
            recorded_tag_position = self.recorded_tag_position, 
            recorded_tag_orientation =self.recorded_tag_orientation
            )

    def load(self, filename):
        data = np.load(str(pathlib.Path().resolve()) + '/recordings/' + str(filename) + '.npz')
        return data





