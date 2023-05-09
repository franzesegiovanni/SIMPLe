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
from pynput.keyboard import Listener, KeyCode
from franka_gripper.msg import GraspActionGoal, HomingActionGoal, StopActionGoal
from utils import get_quaternion_from_euler
class Panda:
    
    def __init__(self, rec_frequency,control_frequency, ff_pos, ff_stiff_lin, arm_id=''):
        # ff = feedback_factor
    
        self.rec_freq = rec_frequency
        self.control_freq = control_frequency

        # The execution factor is a multiplier used to accelerate or slow down an execution
        self.execution_factor = 1

        self.name = arm_id

        # Distance threshold of the current cart position and the first point of the trajectory.
        # If greater than the threshold the trajectory will not start
        self.start_safety_threshold = 0.3

        self.set_K = dynamic_reconfigure.client.Client('/dynamic_reconfigure_compliance_param_node', config_callback=None)
        self.K_ori = 30.0
        self.K_cart = 400.0
        self.K_null = 0.0

        self.start = True
        self.end = False

        self.grip_command = GraspActionGoal()
        self.home_command = HomingActionGoal()
        self.stop_command = StopActionGoal()

        # The index variable is updated to allow for easy access to the current index position within the execution trajectory
        self.index = 0

        # Execution variables used to for corrections
        self.execution_traj = np.zeros(3)
        self.execution_ori = np.zeros(3)
        self.execution_stiff_lin = np.zeros(3)
        self.execution_stiff_ori = np.zeros(3)

        self.gripper_width = 0
        self.grip_command.goal.epsilon.inner = 0.1
        self.grip_command.goal.epsilon.outer = 0.1
        self.grip_command.goal.speed = 0.1
        self.grip_command.goal.force = 5
        self.grip_command.goal.width = 1

        self.attractor_distance_threshold = 0.08
        self.trajectory_distance_threshold = 0.08

        self.length_scale = 0.05
        self.correction_window = 50
        self.correction_mode = 0

        self.feedback = np.zeros(3)
        self.offset = [0, 0, 0, 0, 0, 0]
        self.btn = np.zeros(2)
        self.enable_corr = False
        self.completed = True
        self.pause = False
        self.recording = False
        self.external_record = True

        self.feedback_factor_pos=0.01
        self.feedback_factor_ori=0.001
        self.feedback_factor_stiff_lin=1
        self.feedback_factor_stiff_ori=0.1

        self.mu_index=0.0
        rospy.Subscriber("/panda_dual/bimanual_cartesian_impedance_controller/" + str(self.name) + "_cartesian_pose",
                         PoseStamped, self.ee_pose_callback)
        rospy.Subscriber("panda_dual/" + str(self.name) + "_state_controller/joint_states", JointState,
                         self.joint_callback)
        rospy.Subscriber("/" + str(self.name) + "_franka_gripper/joint_states", JointState, self.gripper_callback)
        rospy.Subscriber("/" + str(self.name) + "/stiffness", Float32MultiArray, self.stiffness_callback, queue_size=1)

        self.goal_pub = rospy.Publisher(
            "/panda_dual/bimanual_cartesian_impedance_controller/" + str(self.name) + "_equilibrium_pose", PoseStamped,
            queue_size=0)
        self.configuration_pub = rospy.Publisher(
            "panda_dual/bimanual_cartesian_impedance_controller/" + str(self.name) + "_nullspace", JointState,
            queue_size=0)
        # self.gripper_pub = rospy.Publisher(str(self.name)+ "_gripper",Float32, queue_size=0)
        self.gripper_pub = rospy.Publisher("/" + str(self.name) + "_franka_gripper/grasp/goal", GraspActionGoal,
                                           queue_size=0)
        self.homing_pub = rospy.Publisher("/" + str(self.name) + "_franka_gripper/homing/goal", HomingActionGoal,
                                          queue_size=0)
        self.stop_pub = rospy.Publisher("/" + str(self.name) + "_franka_gripper/stop/goal", StopActionGoal,
                                          queue_size=0)

        self.stiffness_pub = rospy.Publisher("/" + str(self.name) + "/stiffness", Float32MultiArray,
                                          queue_size=0)

        self.nullspace_stiffness_pub = rospy.Publisher("/panda_dual/bimanual_cartesian_impedance_controller/" + str(self.name) + "_nullspace", JointState,
                                          queue_size=0)

        rospy.Subscriber("panda_dual/" + str(self.name) + "/goto", PoseStamped, self.go_to_3d)
        rospy.Subscriber("panda_dual/" + str(self.name) + "/execute", Bool, self.execute)

        self.goto_pub = rospy.Publisher("panda_dual/" + str(self.name) + "/goto", PoseStamped, queue_size=0)
        self.execute_pub = rospy.Publisher("panda_dual/" + str(self.name) + "/execute", Bool, queue_size=0)

        self.listener = Listener(on_press=self._on_press)
        self.listener.start()

    def _on_press(self, key):
        # This function runs on the background and checks if a keyboard key was pressed
        if key == KeyCode.from_char('e'):
            self.end = True

    def ee_pose_callback(self, data):
        self.cart_pos = [data.pose.position.x, data.pose.position.y, data.pose.position.z]
        self.cart_ori = [data.pose.orientation.w, data.pose.orientation.x, data.pose.orientation.y,
                         data.pose.orientation.z]

    def stiffness_callback(self, stiffness):
        self.set_stiffness(stiffness.data[0],stiffness.data[1],stiffness.data[2],stiffness.data[3],stiffness.data[4],stiffness.data[5],stiffness.data[6])

    # joint angle subscriber
    def joint_callback(self, data):
        self.joint_pos = data.position[0:7]

    # gripper state subscriber
    def gripper_callback(self, data):
        self.gripper_width = np.copy(data.position[0] + data.position[1])

    def move_gripper(self, width):
        if width < 0.07 and self.grip_command.goal.width != 0:
            self.grip_command.goal.width = 0
            self.gripper_pub.publish(self.grip_command)

        elif width > 0.07 and self.grip_command.goal.width != 1:
            self.grip_command.goal.width = 1
            self.gripper_pub.publish(self.grip_command)

    def home_gripper(self):
        self.homing_pub.publish(self.home_command)

    def stop_gripper(self):
        self.stop_pub.publish(self.stop_command)


    def set_stiffness(self, k_t1, k_t2, k_t3, k_r1, k_r2, k_r3, k_ns):

        self.set_K.update_configuration({str(self.name) + "_translational_stiffness_X": k_t1})
        self.set_K.update_configuration({str(self.name) + "_translational_stiffness_Y": k_t2})
        self.set_K.update_configuration({str(self.name) + "_translational_stiffness_Z": k_t3})
        self.set_K.update_configuration({str(self.name) + "_rotational_stiffness_X": k_r1})
        self.set_K.update_configuration({str(self.name) + "_rotational_stiffness_Y": k_r2})
        self.set_K.update_configuration({str(self.name) + "_rotational_stiffness_Z": k_r3})
        # self.set_K.update_configuration({str(self.name) + "_nullspace_stiffness": k_ns})

    def Active(self):
        self.set_stiffness(400.0, 400.0, 400.0, 30.0, 30.0, 30.0, 0.0)

    def Passive(self):
        self.set_stiffness(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

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

    def execute_test(self):
        # print('check')
        self.update_params()

        self.pause = False
        
        if 1:
            self.completed = False
            self.end = False
    
            self.execution_traj = np.asarray(self.recorded_traj)
            self.execution_ori = np.asarray(self.recorded_ori)
            self.execution_gripper = np.asarray(self.recorded_gripper)
            self.execution_stiff_lin = np.asarray(self.recorded_stiffness_lin)
            self.execution_stiff_ori = np.asarray(self.recorded_stiffness_ori)
            
            i = 0
            self.index = 0

            i_position = [self.execution_traj[0][0], self.execution_traj[1][0], self.execution_traj[2][0]]

            if self.start_safety_check(i_position) == False:
                return            
            
            self.set_stiffness(self.K_cart, self.K_cart, self.K_cart, self.K_ori, self.K_ori, self.K_ori, 0.0)

            if self.external_record:
                rospy.set_param("/dual_teaching/recording", True)
            
            i, attractor_pos, attractor_ori, beta= self.GGP()

            while 1: #i < (self.execution_traj.shape[1])-1:
                #print(i)
                self.execution_factor = rospy.get_param("/dual_teaching/execution_factor")
                r = rospy.Rate(self.control_freq*self.execution_factor)

                while self.pause:
                    r.sleep()

                if (np.sum(self.offset) != 0 and self.correction_mode==1):
                    i, attractor_pos, attractor_ori, beta= self.teleoperate(attractor_pos, attractor_ori)
                else:    
                    i, attractor_pos, attractor_ori, beta= self.GGP()
                if self.end:
                    break
                stiff_msg = Float32MultiArray()
                stiff_msg.data =beta * np.array([self.execution_stiff_lin[0][i], self.execution_stiff_lin[1][i], self.execution_stiff_lin[2][i], self.execution_stiff_ori[0][i],self.execution_stiff_ori[1][i],self.execution_stiff_ori[2][i], 0.0]).astype(np.float32)
                self.stiffness_pub.publish(stiff_msg)
                self.set_attractor(attractor_pos, attractor_ori)
                self.move_gripper(self.execution_gripper[0, i])
                r.sleep()
    def execute(self, start):
        # print('check')
        self.update_params()

        self.pause = False
        self.set_stiffness(self.K_cart, self.K_cart, self.K_cart, self.K_ori, self.K_ori, self.K_ori, 0)

        if start.data is True:
            self.completed = False
            self.end = False
    
            self.execution_traj = np.asarray(self.recorded_traj)
            self.execution_ori = np.asarray(self.recorded_ori)
            self.execution_gripper = np.asarray(self.recorded_gripper)
            self.execution_stiff_lin = np.asarray(self.recorded_stiffness_lin)
            self.execution_stiff_ori = np.asarray(self.recorded_stiffness_ori)
            
            i = 0
            self.index = 0

            i_position = [self.execution_traj[0][0], self.execution_traj[1][0], self.execution_traj[2][0]]

            if self.start_safety_check(i_position) == False:
                return            
            

            if self.external_record:
                rospy.set_param("/dual_teaching/recording", True)

            self.mu_index=0.0

            i, attractor_pos, attractor_ori, beta= self.GGP()

            while 1: #i < (self.execution_traj.shape[1])-1:
                #print(i)
                self.execution_factor = rospy.get_param("/dual_teaching/execution_factor")
                r = rospy.Rate(self.control_freq*self.execution_factor)

                while self.pause:
                    r.sleep()

                if (np.sum(self.offset) != 0 and self.correction_mode==1):
                    i, attractor_pos, attractor_ori, beta= self.teleoperate(attractor_pos, attractor_ori)
                else:    
                    i, attractor_pos, attractor_ori, beta= self.GGP()
                if self.end:
                    break
                stiff_msg = Float32MultiArray()
                stiff_msg.data =beta * np.array([self.execution_stiff_lin[0][i], self.execution_stiff_lin[1][i], self.execution_stiff_lin[2][i], self.execution_stiff_ori[0][i],self.execution_stiff_ori[1][i],self.execution_stiff_ori[2][i], 0.0]).astype(np.float32)
                self.stiffness_pub.publish(stiff_msg)
                self.set_attractor(attractor_pos, attractor_ori)
                self.move_gripper(self.execution_gripper[0, i])
                r.sleep()
                

            if self.external_record:
                rospy.set_param("/dual_teaching/recording", False)

            if not self.recording: #if self.recording is on, the trajectory if overwritten after the execution
                self.recorded_traj = np.asarray(self.execution_traj)
                self.recorded_ori = np.asarray(self.execution_ori)
                self.recorded_gripper = np.asarray(self.execution_gripper)
                self.recorded_stiffness_lin = np.asarray(self.execution_stiff_lin)
                self.recorded_stiffness_ori = np.asarray(self.execution_stiff_ori)
        
        start.data = False
        self.completed = True
        print('end_trajactory')


    def GGP(self):
        look_ahead=5 # how many steps forward is the attractor for any element of the graph

        labda_position=0.05
        lamda_time=0.05
        lambda_index=self.rec_freq*lamda_time
        
        sigma_treshold= 1 - np.exp(-2)
        #calcolation of correlation in space
        position_error= np.linalg.norm(self.execution_traj.T - np.array(self.cart_pos), axis=1)/labda_position
        # k_star_position=np.exp(-position_error)

        #calcolation of correlation in time
        index_error= np.abs(np.arange(self.execution_traj.shape[1])-self.index)/lambda_index
        index_error_clip= np.clip(index_error, 0, 1) #1
        # k_star_time= np.exp(-index_error)
        # k_star_time = np.exp(-index_error_clip)

        # Calculate the product of the two correlation vectors
        k_start_time_position=np.exp(-position_error-index_error_clip)#k_star_position*k_star_time

        # Find the element with the maximum correlation
        # mu_index = int(np.argmax(k_start_time_position))
        
        # Compute the uncertainty only as a function of the correlation in space 
        # sigma_position= 1- np.max(k_star_position)

        # Compute the uncertainty only as a function of the correlation in space and time
        sigma_position_time= 1- np.max(k_start_time_position)

        # Compute the scaling factor for the stiffness Eq 15
        if sigma_position_time > sigma_treshold: 
            beta= (1-sigma_position_time)/(1-sigma_treshold)
            self.mu_index = int(np.argmax(k_start_time_position))
        else:
            beta=1
            # print(self.mu_index)
            self.mu_index = int(self.mu_index+ 1.0*np.sign((int(np.argmax(k_start_time_position))- self.mu_index)))

        # # Compute the scaling factor for the stiffness Eq 15
        # if sigma_position > sigma_treshold: 
        #     beta= (1-sigma_position)/(1-sigma_treshold)
        # else:
        #     beta=1
        # print(int(np.round(look_ahead*self.execution_factor)))
        i = np.min([self.mu_index+int(np.round(look_ahead*self.execution_factor)), self.execution_traj.shape[1]-1]) 
        self.index=np.min([self.mu_index+look_ahead, self.execution_traj.shape[1]-1])

        attractor_pos = [self.execution_traj[0][i], self.execution_traj[1][i], self.execution_traj[2][i]]
        attractor_ori = [self.execution_ori[0][i], self.execution_ori[1][i], self.execution_ori[2][i],self.execution_ori[3][i]]

        # print(i)
        return  i, attractor_pos, attractor_ori, beta

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
        self.set_stiffness(self.K_cart, self.K_cart, self.K_cart, self.K_ori, self.K_ori, self.K_ori, 0)

        for i in range(step_num):
            position = [x[i], y[i], z[i]]
            quat = np.slerp_vectorized(q_start, q_goal, i / step_num)
            orientation = [quat.w, quat.x, quat.y, quat.z]
            self.set_attractor(position, orientation)
            r.sleep()

    def Kinesthetic_Demonstration(self, trigger=0.005, active=False):
        self.update_params()
        self.recording = True
        self.pause = False
        time.sleep(1)
        r = rospy.Rate(self.control_freq*self.execution_factor)
        stiff_msg = Float32MultiArray()
        if not active:
            stiff_msg.data = np.array([0, 0, 0, 0, 0, 0, 0]).astype(np.float32)
            self.stiffness_pub.publish(stiff_msg)

        self.end = False
        init_pos = self.cart_pos
        vel = 0
        print("Move robot to start recording.")
        # while vel < trigger:
        #     vel = math.sqrt((self.cart_pos[0] - init_pos[0]) ** 2 + (self.cart_pos[1] - init_pos[1]) ** 2 + (
        #             self.cart_pos[2] - init_pos[2]) ** 2)

        print("Recording started. Press e to stop and press u to pause/continue.")

        self.recorded_traj = self.cart_pos
        self.recorded_ori = self.cart_ori
        self.recorded_joint = self.joint_pos
        self.recorded_gripper = self.gripper_width
        self.recorded_stiffness_lin=[self.K_cart, self.K_cart, self.K_cart]
        self.recorded_stiffness_ori=[self.K_ori, self.K_ori, self.K_ori]

        if self.external_record:
            rospy.set_param("/dual_teaching/recording", True)

        while not self.end:

            self.recorded_traj = np.c_[self.recorded_traj, self.cart_pos]
            self.recorded_ori = np.c_[self.recorded_ori, self.cart_ori]
            self.recorded_joint = np.c_[self.recorded_joint, self.joint_pos]
            self.recorded_gripper = np.c_[self.recorded_gripper, self.gripper_width]
            self.recorded_stiffness_lin=np.c_[self.recorded_stiffness_lin, [self.K_cart, self.K_cart, self.K_cart]]
            self.recorded_stiffness_ori=np.c_[self.recorded_stiffness_ori, [self.K_ori, self.K_ori, self.K_ori]]

            if self.pause:
                self.set_attractor(self.cart_pos, self.cart_ori)

                self.move_gripper(self.recorded_gripper[0, -1])
            
                stiff_msg = Float32MultiArray()
                stiff_msg.data = np.array([600, 600, 600, 30, 30, 30, 0]).astype(np.float32)
                self.stiffness_pub.publish(stiff_msg)

                while self.pause:
                    if self.end:
                        break
                    r.sleep()   

                stiff_msg = Float32MultiArray()
                stiff_msg.data = np.array([0, 0, 0, 0, 0, 0, 0]).astype(np.float32)
                self.stiffness_pub.publish(stiff_msg)
                self.stop_gripper()

            r.sleep()
        
        self.recording = False

        if self.external_record:
            rospy.set_param("/dual_teaching/recording", False)
            
    def save(self, data='last'):
        np.savez(str(pathlib.Path().resolve()) + '/data/' + str(self.name) + '_' + str(data) + '.npz',
                 recorded_traj=self.recorded_traj,
                 recorded_ori=self.recorded_ori,
                 recorded_gripper=self.recorded_gripper,
                 recorded_stiffness_lin=self.recorded_stiffness_lin,
                 recorded_stiffness_ori=self.recorded_stiffness_ori)

    def load(self, file='last'):
        data = np.load(str(pathlib.Path().resolve()) + '/data/' + str(self.name) + '_' + str(file) + '.npz')

        self.recorded_traj = data['recorded_traj'],
        self.recorded_ori = data['recorded_ori'],
        self.recorded_gripper = data['recorded_gripper'],
        self.recorded_stiffness_lin = data['recorded_stiffness_lin'],
        self.recorded_stiffness_ori = data['recorded_stiffness_ori'],
        self.recorded_traj = self.recorded_traj[0]
        self.recorded_ori = self.recorded_ori[0]
        self.recorded_gripper = self.recorded_gripper[0]
        self.recorded_stiffness_lin = self.recorded_stiffness_lin[0]
        self.recorded_stiffness_ori = self.recorded_stiffness_ori[0]

    def continue_traj(self, index, traj):
        if (np.linalg.norm(np.array(self.cart_pos)-traj[:, index])) <= self.attractor_distance_threshold:
            return True
        else:
            return False
     
    def square_exp(self, lengthscale, x_1, x_2):
        d = np.linalg.norm(x_1-x_2,2)
        return np.exp(-d ** 2 /(2*lengthscale ** 2))


    def correct_attractor(self, index, traj_pos, traj_ori, magnitude):

        for j in range(int(max(0, index - self.correction_window)),
                    int(min(index + self.correction_window, traj.shape[1]-1))):

            square_exp= self.square_exp(self.length_scale, traj[:,j], traj[:,index])

            delta_x = magnitude*self.offset[0] * square_exp
            delta_y = magnitude*self.offset[1] * square_exp
            delta_z = magnitude*self.feedback[2] * square_exp

            q_goal=np.quaternion(traj_ori[0][j], traj_ori[1][j], traj_ori[2][j], traj_ori[3][j])

            q_delta=get_quaternion_from_euler(magnitude*self.offset[3]* square_exp, magnitude*self.offset[4]* square_exp, magnitude*self.offset[5]* square_exp)

            q_delta_quaternion=np.quaternion(q_delta[0],q_delta[1],q_delta[2],q_delta[3])

            # panda_right_q_delta=from_euler_angles(panda_right_alpha_beta_gamma)  
            q_goal=q_delta_quaternion*q_goal

            traj_pos[0][j] += delta_x
            traj_pos[1][j] += delta_y  
            traj_pos[2][j] += delta_z
            

            traj_ori[0][j]= q_goal.w
            traj_ori[1][j]= q_goal.x
            traj_ori[2][j]= q_goal.y
            traj_ori[3][j]= q_goal.z


        self.offset = np.zeros(6)

        return traj_pos, traj_ori

    def correct_stiffness(self, index, traj, stiff,  magnitude):

        for j in range(int(max(0, index - self.correction_window)),
                       int(min(index + self.correction_window, traj.shape[1]-1))):
            delta_K_x = magnitude*np.abs(self.feedback[0]) * self.square_exp(self.length_scale, traj[:,j], traj[:,index])
            delta_K_y = magnitude*np.abs(self.feedback[1]) * self.square_exp(self.length_scale, traj[:,j], traj[:,index])
            delta_K_z = magnitude*np.abs(self.feedback[2]) * self.square_exp(self.length_scale, traj[:,j], traj[:,index])

            stiff[0][j] += delta_K_x
            stiff[1][j] += delta_K_y
            stiff[2][j] += delta_K_z

        self.feedback = np.zeros(3)
        return stiff


    def update_params(self):
        self.length_scale = rospy.get_param("/dual_teaching/correction_length_scale")
        self.correction_window = rospy.get_param("/dual_teaching/correction_window_size")
        self.execution_factor = rospy.get_param("/dual_teaching/execution_factor")

        self.attractor_distance_threshold = rospy.get_param("/dual_teaching/attractor_distance_threshold")
        self.trajectory_distance_threshold = rospy.get_param("/dual_teaching/trajectory_distance_threshold")

        self.feedback_factor_pos = rospy.get_param(f"/dual_teaching/{self.name}_feedback_factor_position")
        self.enable_corr = rospy.get_param(f"/dual_teaching/{self.name}_enable_correction")
    
    def start_safety_check(self, t_pos):
        if np.linalg.norm(np.array(self.cart_pos)-np.array(t_pos)) > self.start_safety_threshold:
            print(f"{self.name} is too far from the start position, please make sure the go_to_start function has been run")
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
        self.nullspace_stiffness_pub.publish(ns_msg)
        self.set_K.update_configuration({f'{str(self.name)}_nullspace_stiffness':10})

        rospy.sleep(rospy.Duration(secs=5))

        self.set_K.update_configuration({f'{str(self.name)}_nullspace_stiffness':0})
