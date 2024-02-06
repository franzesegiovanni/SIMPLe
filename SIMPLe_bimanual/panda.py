"""
Authors: Giovanni Franzese, June 2022
Email: g.franzese@tudelft.nl
Cognitive Robotics, TU Delft
This code is part of TERI (TEaching Robots Interactively) project
"""
#%%
# !/usr/bin/env python
import rospy
import math
import numpy as np
import time
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import dynamic_reconfigure.client
from std_msgs.msg import Float32MultiArray, Bool
import pathlib
from pynput.keyboard import Listener, Key
from franka_gripper.msg import GraspActionGoal, HomingActionGoal, StopActionGoal
from SIMPLe_bimanual.ggp import GGP
class Panda:
    
    def __init__(self, control_frequency=30, arm_id=''):
    
        self.control_freq = control_frequency

        self.name = arm_id

        # Distance threshold of the current cart position and the first point of the trajectory.
        # If greater than the threshold the trajectory will not start
        self.start_safety_threshold = 0.3

        self.set_K = dynamic_reconfigure.client.Client('/dynamic_reconfigure_compliance_param_node', config_callback=None)
        self.K_ori = 30.0
        self.K_cart = 400.0
        self.K_null = 0.0

        self.end = False

        self.grip_command = GraspActionGoal()
        self.home_command = HomingActionGoal()
        self.gripper_width_close=0.07 # if the recorded gripper is lower than 0.05, the robot is going to close during execution and if it larger, it is going to open
        
        self.grip_command.goal.epsilon.inner = 0.3 #by having this big tollerance, the robot will adapt the grasp to any object dimension 
        self.grip_command.goal.epsilon.outer = 0.3 #by having this big tollerance, the robot will adapt the grasp to any object dimension 
        self.grip_command.goal.speed = 1
        self.grip_command.goal.force = 1
        self.grip_command.goal.width = 1

        self.attractor_distance_threshold = 0.08
        self.trajectory_distance_threshold = 0.08

        rospy.Subscriber("/panda_dual/bimanual_cartesian_impedance_controller/" + str(self.name) + "_cartesian_pose",
                         PoseStamped, self.ee_pose_callback)
        rospy.Subscriber("panda_dual/" + str(self.name) + "_state_controller/joint_states", JointState,
                         self.joint_callback)
        rospy.Subscriber("/" + str(self.name) + "_franka_gripper/joint_states", JointState, self.gripper_callback)

        self.goal_pub = rospy.Publisher(
            "/panda_dual/bimanual_cartesian_impedance_controller/" + str(self.name) + "_equilibrium_pose", PoseStamped,
            queue_size=0)
        self.configuration_pub = rospy.Publisher(
            "panda_dual/bimanual_cartesian_impedance_controller/" + str(self.name) + "_nullspace", JointState,
            queue_size=0)

        self.gripper_pub = rospy.Publisher("/" + str(self.name) + "_franka_gripper/grasp/goal", GraspActionGoal,
                                           queue_size=0)
        self.homing_pub = rospy.Publisher("/" + str(self.name) + "_franka_gripper/homing/goal", HomingActionGoal,
                                          queue_size=0)
        self.stop_pub = rospy.Publisher("/" + str(self.name) + "_franka_gripper/stop/goal", StopActionGoal,
                                          queue_size=0)

        self.nullspace_configuration_pub = rospy.Publisher("/panda_dual/bimanual_cartesian_impedance_controller/" + str(self.name) + "_nullspace", JointState,
                                          queue_size=0)

        rospy.Subscriber("panda_dual/" + str(self.name) + "/goto", PoseStamped, self.go_to_3d)
        rospy.Subscriber("panda_dual/" + str(self.name) + "/execute", Bool, self.execute)

        self.goto_pub = rospy.Publisher("panda_dual/" + str(self.name) + "/goto", PoseStamped, queue_size=0)
        self.execute_pub = rospy.Publisher("panda_dual/" + str(self.name) + "/execute", Bool, queue_size=0)

        self.listener = Listener(on_press=self._on_press)
        self.listener.start()

    def _on_press(self, key):
        # This function runs on the background and checks if a keyboard key was pressed
        if key == Key.esc:
            self.end = True

    def ee_pose_callback(self, data):
        self.cart_pos = [data.pose.position.x, data.pose.position.y, data.pose.position.z]
        self.cart_ori = [data.pose.orientation.w, data.pose.orientation.x, data.pose.orientation.y,
                         data.pose.orientation.z]

    # joint angle subscriber
    def joint_callback(self, data):
        self.joint_pos = data.position[0:7]

    # gripper state subscriber
    def gripper_callback(self, data):
        self.gripper_width = np.copy(data.position[0] + data.position[1])

    def move_gripper(self, width):
        if width < self.gripper_width_close and self.grip_command.goal.width != 0:
            self.grip_command.goal.width = 0
            self.gripper_pub.publish(self.grip_command)

        elif width > self.gripper_width_close and self.grip_command.goal.width != 1:
            self.grip_command.goal.width = 1
            self.gripper_pub.publish(self.grip_command)

    def home_gripper(self):
        self.homing_pub.publish(self.home_command)

    def set_stiffness(self, k_t1, k_t2, k_t3, k_r1, k_r2, k_r3):

        self.set_K.update_configuration({str(self.name) + "_translational_stiffness_X": k_t1})
        self.set_K.update_configuration({str(self.name) + "_translational_stiffness_Y": k_t2})
        self.set_K.update_configuration({str(self.name) + "_translational_stiffness_Z": k_t3})
        self.set_K.update_configuration({str(self.name) + "_rotational_stiffness_X": k_r1})
        self.set_K.update_configuration({str(self.name) + "_rotational_stiffness_Y": k_r2})
        self.set_K.update_configuration({str(self.name) + "_rotational_stiffness_Z": k_r3})

    def Active(self):
        self.set_stiffness(400.0, 400.0, 400.0, 30.0, 30.0, 30.0)

    def Passive(self):
        self.set_stiffness(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    def set_attractor(self, pos, quat):
        goal = PoseStamped()
        goal.header.seq = 1
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"
        goal.pose.position.x = pos[0]
        goal.pose.position.y = pos[1]
        goal.pose.position.z = pos[2]

        goal.pose.orientation.w = quat[0]
        goal.pose.orientation.x = quat[1]
        goal.pose.orientation.y = quat[2]
        goal.pose.orientation.z = quat[3]

        self.goal_pub.publish(goal)

    def set_configuration(self, joint):
        joint_des = Float32MultiArray()
        joint_des.data = np.array(joint).astype(np.float32)
        self.configuration_pub.publish(joint_des)

    def execute_traj(self):
        goal = Bool()
        goal.data = True
        self.execute_pub.publish(goal)

    def execute(self, start): #call back function that executes the trajectory using a Graph Gaussian Process

        self.set_stiffness(self.K_cart, self.K_cart, self.K_cart, self.K_ori, self.K_ori, self.K_ori)

        if start.data is True:
            self.end = False

            traj_starting_position = [self.recorded_traj[0][0], self.recorded_traj[1][0], self.recorded_traj[2][0]]

            if self.start_safety_check(traj_starting_position) == False:
                return            


            ggp=GGP(execution_traj=self.recorded_traj)

            r = rospy.Rate(self.control_freq)
            
            while not self.end:


   
                i, beta= ggp.step(cart_pos=self.cart_pos)

                attractor_pos = [self.recorded_traj[0][i], self.recorded_traj[1][i], self.recorded_traj[2][i]]
                attractor_ori = [self.recorded_ori[0][i], self.recorded_ori[1][i], self.recorded_ori[2][i],self.recorded_ori[3][i]]

                self.set_stiffness(beta *self.recorded_stiff_lin[0][i], beta *self.recorded_stiff_lin[1][i], beta *self.recorded_stiff_lin[2][i], beta *self.recorded_stiff_ori[0][i], beta *self.recorded_stiff_ori[1][i], beta *self.recorded_stiff_ori[2][i])
                self.set_attractor(attractor_pos, attractor_ori)
                self.move_gripper(self.recorded_gripper[0, i])
                r.sleep()
        
        start.data = False
        print('Stopped execution of the behaviour of' + str(self.name))



    def go_to_start(self):
        goal = PoseStamped()
        goal.header.seq = 1
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"
        goal.pose.position.x = self.recorded_traj[0][0]
        goal.pose.position.y = self.recorded_traj[1][0]
        goal.pose.position.z = self.recorded_traj[2][0]

        goal.pose.orientation.w = self.recorded_ori[0][0]
        goal.pose.orientation.x = self.recorded_ori[1][0]
        goal.pose.orientation.y = self.recorded_ori[2][0]
        goal.pose.orientation.z = self.recorded_ori[3][0]

        self.goto_pub.publish(goal)

    def go_to_3d(self, data):
        control_freq = 50
        start = self.cart_pos
        start_ori = self.cart_ori
        r = rospy.Rate(control_freq)
        # interpolate from start to goal with attractor distance of approx 1 mm
        goal_ = [data.pose.position.x, data.pose.position.y, data.pose.position.z]
        q_start = np.quaternion(start_ori[0], start_ori[1], start_ori[2], start_ori[3])
        q_goal = np.quaternion(data.pose.orientation.w, data.pose.orientation.x, data.pose.orientation.y,
                               data.pose.orientation.z)

        squared_dist = np.sum(np.subtract(start, goal_) ** 2, axis=0)
        dist = np.sqrt(squared_dist)
        interp_dist = 0.001  # [m]
        step_num = math.floor(dist / interp_dist)

        x = np.linspace(start[0], goal_[0], step_num)
        y = np.linspace(start[1], goal_[1], step_num)
        z = np.linspace(start[2], goal_[2], step_num)
        quat = np.slerp_vectorized(q_start, q_goal, 0)

        position = [x[0], y[0], z[0]]
        orientation = [quat.w, quat.x, quat.y, quat.z]

        self.set_attractor(position, orientation)
        self.set_stiffness(self.K_cart, self.K_cart, self.K_cart, self.K_ori, self.K_ori, self.K_ori)

        for i in range(step_num):
            position = [x[i], y[i], z[i]]
            quat = np.slerp_vectorized(q_start, q_goal, i / step_num)
            orientation = [quat.w, quat.x, quat.y, quat.z]
            self.set_attractor(position, orientation)
            r.sleep()

    def Kinesthetic_Demonstration(self, active=False):
        time.sleep(1)
        r = rospy.Rate(self.control_freq)
        if not active:
            self.set_stiffness(0, 0, 0, 0, 0, 0)

        self.end = False

        print("Recording started. Press Esc to stop the recording")

        self.recorded_traj = self.cart_pos
        self.recorded_ori = self.cart_ori
        self.recorded_joint = self.joint_pos
        self.recorded_gripper = self.gripper_width
        self.recorded_stiffness_lin=[self.K_cart, self.K_cart, self.K_cart]
        self.recorded_stiffness_ori=[self.K_ori, self.K_ori, self.K_ori]

        while not self.end:

            self.recorded_traj = np.c_[self.recorded_traj, self.cart_pos]
            self.recorded_ori = np.c_[self.recorded_ori, self.cart_ori]
            self.recorded_joint = np.c_[self.recorded_joint, self.joint_pos]
            self.recorded_gripper = np.c_[self.recorded_gripper, self.gripper_width]
            self.recorded_stiffness_lin=np.c_[self.recorded_stiffness_lin, [self.K_cart, self.K_cart, self.K_cart]]
            self.recorded_stiffness_ori=np.c_[self.recorded_stiffness_ori, [self.K_ori, self.K_ori, self.K_ori]]

            r.sleep()
            
    def save(self, data='last'):
        np.savez(str(pathlib.Path().resolve()) + '/data/' + str(self.name) + '_' + str(data) + '.npz',
                 recorded_traj=self.recorded_traj,
                 recorded_ori=self.recorded_ori,
                 recorded_gripper=self.recorded_gripper,
                 recorded_stiffness_lin=self.recorded_stiffness_lin,
                 recorded_stiffness_ori=self.recorded_stiffness_ori)

    def load(self, file='last'):
        data = np.load(str(pathlib.Path().resolve()) + '/data/' + str(self.name) + '_' + str(file) + '.npz')

        self.recorded_traj = data['recorded_traj']
        self.recorded_ori = data['recorded_ori']
        self.recorded_gripper = data['recorded_gripper']
        self.recorded_stiffness_lin = data['recorded_stiffness_lin']
        self.recorded_stiffness_ori = data['recorded_stiffness_ori']

    def continue_traj(self, index, traj):
        if (np.linalg.norm(np.array(self.cart_pos)-traj[:, index])) <= self.attractor_distance_threshold:
            return True
        else:
            return False
     
    def start_safety_check(self, t_pos):
        if np.linalg.norm(np.array(self.cart_pos)-np.array(t_pos)) > self.start_safety_threshold:
            print(f"{self.name} is too far from the start position, please make sure the go_to_start function has been run. This is for safety reasons.")
            return False
        else:
            return True

    def home(self):
        goal = PoseStamped()
        goal.header.seq = 1
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"
        goal.pose.position.x = 0.6
        goal.pose.position.y = 0
        goal.pose.position.z = 0.4

        goal.pose.orientation.w = 0
        goal.pose.orientation.x = 1
        goal.pose.orientation.y = 0
        goal.pose.orientation.z = 0

        

        ns_msg = JointState()   
        ns_msg.position = [0, 0, 0, -2.4, 0, 2.4, 0]

        self.goto_pub.publish(goal)
        self.nullspace_configuration_pub.publish(ns_msg)
        self.set_K.update_configuration({f'{str(self.name)}_nullspace_stiffness':10})

        rospy.sleep(rospy.Duration(secs=5))

        self.set_K.update_configuration({f'{str(self.name)}_nullspace_stiffness':0})
