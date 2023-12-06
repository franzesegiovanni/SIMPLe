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
import quaternion
from geometry_msgs.msg import  PoseStamped
import dynamic_reconfigure.client
from std_msgs.msg import Float32MultiArray
import pathlib
from pynput.keyboard import Listener, KeyCode
from .panda import Panda

class DualPanda:
    def __init__(self, arm_id_right='panda_right', arm_id_left='panda_left'):
        rospy.init_node('DualArmControl', anonymous=True)
        self.client = dynamic_reconfigure.client.Client("dual_teaching", timeout=None, config_callback=None)
        
        self.control_freq = 30
        self.rec_freq = 30

        # The execution factor is a multiplier used to accelerate or slow down an execution
        self.execution_factor = 1
        self.client.update_configuration(self.client.update_configuration({"execution_factor": self.execution_factor}))

        # Factors that multiply the feedback from the Spacemouse to allow for more rough or granular control
        self.feedback_factor_pos = 0.02
        self.feedback_factor_ori = 0.02
        self.feedback_factor_stiffness = 0.05

        self.target_coupling_stiffness = 800

        self.Panda_right = Panda(self.rec_freq, self.control_freq, self.feedback_factor_pos, self.feedback_factor_stiffness, arm_id_right)
        self.Panda_left = Panda(self.rec_freq, self.control_freq, self.feedback_factor_pos, self.feedback_factor_stiffness, arm_id_left)
        
        self.coupling_diff_pub = rospy.Publisher("/panda_dual/bimanual_cartesian_impedance_controller/equilibrium_distance", PoseStamped, queue_size=0)
      
        # Start keyboard listener
        self.listener = Listener(on_press=self._on_press)
        self.listener.start()

        # enable_correction is used to enable/disable corrections using the spacemouse
        self.enable_correction = False

        self.external_record = True

        # The index variable is updated to allow for easy access to the current index position within the execution trajectory
        self.index = 0

        self.end = True

    def _on_press(self, key):
        """
        Function that runs in the background and checks for a key press
        """
        if key == KeyCode.from_char('e'):
            self.end = True
            self.Panda_left.end = True
            self.Panda_right.end = True

        if key == KeyCode.from_char('u') and (self.Panda_left.recording or not self.Panda_left.completed):
            self.Panda_left.pause = not self.Panda_left.pause
            print(f"Panda left pause toggled, state: {self.Panda_left.pause}")

        if key == KeyCode.from_char('i') and (self.Panda_right.recording or not self.Panda_right.completed):
            self.Panda_right.pause = not self.Panda_right.pause
            print(f"Panda right toggled, state: {self.Panda_right.pause}")

    def Kinesthetic_Demonstration_BiManual(self, trigger=0.005, active=False, verbose=True):
        """
        Funtion to record a bimanual demonstration.

        :param trigger: The offeset threshold for which the demonstration start is detected
        :param active: A boolean
        that determines the stiffness of the arms. If you have Active equal to true, you are recording but the
        manipulators' stiffness is not dropped to zero. You can for example learn to synchronize what two
        manipulators are doing
        :param verbose: Boolean that determines if the function prints to the terminal
        """
        self.execution_factor = rospy.get_param("/dual_teaching/execution_factor")

        self.Panda_left.update_params()
        self.Panda_right.update_params()
    
        r = rospy.Rate(self.control_freq*self.execution_factor)
        
        if active is False:
            self.Panda_left.Passive()
            self.Panda_right.Passive()
        else:
            self.Panda_left.Active()
            self.Panda_right.Active()

        self.end = False

        init_pos_right = self.Panda_right.cart_pos
        init_pos_left = self.Panda_left.cart_pos

        vel_right = 0
        vel_left = 0

        if verbose:
            print("Move robot to start recording.")

        # while vel_right < trigger and vel_left < trigger:
        #     vel_right = math.sqrt((self.Panda_right.cart_pos[0] - init_pos_right[0]) ** 2 + (
        #             self.Panda_right.cart_pos[1] - init_pos_right[1]) ** 2 + (
        #                                   self.Panda_right.cart_pos[2] - init_pos_right[2]) ** 2)
        #     vel_left = math.sqrt((self.Panda_left.cart_pos[0] - init_pos_left[0]) ** 2 + (
        #             self.Panda_left.cart_pos[1] - init_pos_left[1]) ** 2 + (
        #                                  self.Panda_left.cart_pos[2] - init_pos_left[2]) ** 2)
        if verbose:
            print("Recording started. Press e to stop.")
        rospy.set_param("/dual_teaching/recording", True)
        self.recorded_traj_dual = np.r_[self.Panda_right.cart_pos, self.Panda_left.cart_pos]
        self.recorded_joint_dual = np.r_[self.Panda_right.joint_pos, self.Panda_left.joint_pos]
        self.recorded_ori_dual = np.r_[self.Panda_right.cart_ori, self.Panda_left.cart_ori]
        self.recorded_gripper_dual = np.array([self.Panda_right.gripper_width, self.Panda_left.gripper_width])
        self.recorded_stiffness_lin_dual = np.array([self.Panda_right.K_cart, self.Panda_right.K_cart, self.Panda_right.K_cart, self.Panda_left.K_cart, self.Panda_left.K_cart, self.Panda_left.K_cart])
        self.recorded_stiffness_ori_dual = np.array([self.Panda_right.K_ori, self.Panda_right.K_ori, self.Panda_right.K_ori, self.Panda_left.K_ori, self.Panda_left.K_ori, self.Panda_left.K_ori])
        self.end = False
        while (not self.end): #and (not self.enable_correction*self.Panda_left.completed*self.Panda_right.completed):
            self.recorded_traj_dual = np.c_[
                self.recorded_traj_dual, np.r_[self.Panda_right.cart_pos, self.Panda_left.cart_pos]]
            self.recorded_ori_dual = np.c_[
                self.recorded_ori_dual, np.r_[self.Panda_right.cart_ori, self.Panda_left.cart_ori]]
            self.recorded_joint_dual = np.c_[
                self.recorded_joint_dual, np.r_[self.Panda_right.joint_pos, self.Panda_left.joint_pos]]
            self.recorded_gripper_dual = np.c_[
                self.recorded_gripper_dual, np.array([self.Panda_right.gripper_width, self.Panda_left.gripper_width])]
            self.recorded_stiffness_lin_dual = np.c_[
                self.recorded_stiffness_lin_dual, np.array([self.Panda_right.K_cart, self.Panda_right.K_cart, self.Panda_right.K_cart, 
                self.Panda_left.K_cart, self.Panda_left.K_cart, self.Panda_left.K_cart])]
            self.recorded_stiffness_ori_dual = np.c_[
                self.recorded_stiffness_ori_dual, np.array([self.Panda_right.K_ori, self.Panda_right.K_ori, self.Panda_right.K_ori, 
                self.Panda_left.K_ori, self.Panda_left.K_ori, self.Panda_left.K_ori])]
            r.sleep()

        rospy.set_param("/dual_teaching/recording", False)

    def execute_dual(self, record=False):
        """
        Function to execute a bimanual demonstration. This execution will pause if one of the arms is outside their
        respective attractor distance threshold and thus cannot be used for synchronisation.
        """
        self.execution_factor = rospy.get_param("/dual_teaching/execution_factor")
        r = rospy.Rate(self.control_freq*self.execution_factor)

        self.enable_correction = rospy.get_param("/dual_teaching/dual_enable_correction")
        
        self.Panda_left.update_params()
        self.Panda_right.update_params()

        self.Panda_left.pause = False
        self.Panda_right.pause = False
           
        self.index = 0
        self.Panda_left.index = self.Panda_right.index = self.index

        # Load the recorded trajectories into the execution trajectory to allow for generalised functions
        self.Panda_right.execution_traj = self.recorded_traj_dual[:3]
        self.Panda_right.execution_ori = self.recorded_ori_dual[:4]
        self.Panda_right.execution_gripper = np.c_[self.recorded_gripper_dual[0]].T
        self.Panda_right.execution_stiff_lin = self.recorded_stiffness_lin_dual[:3]
        self.Panda_right.execution_stiff_ori = self.recorded_stiffness_ori_dual[:3]

        self.Panda_left.execution_traj = self.recorded_traj_dual[3:]
        self.Panda_left.execution_ori = self.recorded_ori_dual[4:]
        self.Panda_left.execution_gripper = np.c_[self.recorded_gripper_dual[1]].T
        self.Panda_left.execution_stiff_lin = self.recorded_stiffness_lin_dual[3:]
        self.Panda_left.execution_stiff_ori = self.recorded_stiffness_ori_dual[3:]

        position_right = [self.Panda_right.execution_traj[0][0], self.Panda_right.execution_traj[1][0], self.Panda_right.execution_traj[2][0]]
        orientation_right = [self.Panda_right.execution_ori[0][0], self.Panda_right.execution_ori[1][0], self.Panda_right.execution_ori[2][0],
                           self.Panda_right.execution_ori[3][0]]

        position_left = [self.Panda_left.execution_traj[0][0], self.Panda_left.execution_traj[1][0], self.Panda_left.execution_traj[2][0]]
        orientation_left = [self.Panda_left.execution_ori[0][0], self.Panda_left.execution_ori[1][0], self.Panda_left.execution_ori[2][0],
                               self.Panda_left.execution_ori[3][0]]
        
        if self.Panda_left.start_safety_check(position_left) == False or self.Panda_right.start_safety_check(position_right) == False:
            return

        # TODO Verify if this is wanted in the code as it prevents dual execution with other stiffness params
        prefix = '/dynamic_reconfigure_compliance_param_node'

        self.Panda_left.set_stiffness(rospy.get_param(prefix + '/panda_left_translational_stiffness_X'), 
                                      rospy.get_param(prefix + '/panda_left_translational_stiffness_Y'), 
                                      rospy.get_param(prefix + '/panda_left_translational_stiffness_Z'), 
                                      rospy.get_param(prefix + '/panda_left_rotational_stiffness_X'), 
                                      rospy.get_param(prefix + '/panda_left_rotational_stiffness_Y'), 
                                      rospy.get_param(prefix + '/panda_left_rotational_stiffness_Z'), 
                                      rospy.get_param(prefix + '/panda_left_nullspace_stiffness'))

        self.Panda_right.set_stiffness(rospy.get_param(prefix + '/panda_right_translational_stiffness_X'), 
                                      rospy.get_param(prefix + '/panda_right_translational_stiffness_Y'), 
                                      rospy.get_param(prefix + '/panda_right_translational_stiffness_Z'), 
                                      rospy.get_param(prefix + '/panda_right_rotational_stiffness_X'), 
                                      rospy.get_param(prefix + '/panda_right_rotational_stiffness_Y'), 
                                      rospy.get_param(prefix + '/panda_right_rotational_stiffness_Z'), 
                                      rospy.get_param(prefix + '/panda_right_nullspace_stiffness'))

        # Code to calculate the difference between the arms and publish it to the coupling topic
        d_coupling = np.array(position_right) - np.array(position_left)
        diff_msg = PoseStamped()
        diff_msg.header.seq = 1
        diff_msg.header.stamp = rospy.Time.now()
        diff_msg.pose.position.x = d_coupling[0]
        diff_msg.pose.position.y = d_coupling[1]
        diff_msg.pose.position.z = d_coupling[2]        

        self.coupling_diff_pub.publish(diff_msg)

        rospy.get_param("/dual_teaching/target_coupling_stiffness")
        self.Panda_left.set_K.update_configuration({"coupling_translational_stiffness": self.target_coupling_stiffness})

        if self.external_record:
            rospy.set_param("/dual_teaching/recording", True)

        while self.index < self.recorded_traj_dual.shape[1]:

            while self.Panda_left.pause or self.Panda_right.pause:
                r.sleep()

            self.execution_factor = rospy.get_param("/dual_teaching/execution_factor")
            r = rospy.Rate(self.control_freq*self.execution_factor)


            if self.continue_dual_traj(self.index, self.recorded_traj_dual):

                position_right = [self.Panda_right.execution_traj[0][self.index], self.Panda_right.execution_traj[1][self.index], self.Panda_right.execution_traj[2][self.index]]
                orientation_right = [self.Panda_right.execution_ori[0][self.index], self.Panda_right.execution_ori[1][self.index], self.Panda_right.execution_ori[2][self.index],
                           self.Panda_right.execution_ori[3][self.index]]


                self.Panda_right.set_attractor(position_right, orientation_right)

                self.Panda_right.move_gripper(self.recorded_gripper_dual[0, self.index])

                position_left = [self.Panda_left.execution_traj[0][self.index], self.Panda_left.execution_traj[1][self.index], self.Panda_left.execution_traj[2][self.index]]
                orientation_left = [self.Panda_left.execution_ori[0][self.index], self.Panda_left.execution_ori[1][self.index], self.Panda_left.execution_ori[2][self.index],
                               self.Panda_left.execution_ori[3][self.index]]

                self.Panda_left.set_attractor(position_left, orientation_left)

                self.Panda_left.move_gripper(self.recorded_gripper_dual[1, self.index])

                # Code to calculate the difference between the arms and publish it to the coupling topic
                d_coupling = np.array(position_right) - np.array(position_left)
                diff_msg = PoseStamped()
                diff_msg.header.seq = 1
                diff_msg.header.stamp = rospy.Time.now()
                diff_msg.pose.position.x = d_coupling[0]
                diff_msg.pose.position.y = d_coupling[1]
                diff_msg.pose.position.z = d_coupling[2]
                
                self.coupling_diff_pub.publish(diff_msg)

                self.index += 1
                self.Panda_left.index = self.Panda_right.index = self.index

            r.sleep()


        rospy.set_param("/dual_teaching/recording", False)

        # Set the coupling stiffness back to 0
        self.Panda_left.set_K.update_configuration({"coupling_translational_stiffness": 0})

        
        # Overwrite the recorded trajectories with the execution trajectory as that includes the corrections done during execution
        self.recorded_traj_dual[:3] = self.Panda_right.execution_traj
        self.recorded_traj_dual[3:] = self.Panda_left.execution_traj

        self.recorded_ori_dual[:4] = self.Panda_right.execution_ori
        self.recorded_ori_dual[4:] = self.Panda_left.execution_ori

    def correction_execute_dual(self):
        """
        Function that allows for the execution of a bimanual demonstration while also allowing for corrections from the
        Spacemouse and also kinesthetic corrections. As the trajectory is split and executed separately, it allows for
        synchronisation of the tasks.
        """
        self.Panda_right.recorded_traj = self.recorded_traj_dual[:3]
        self.Panda_right.recorded_ori = self.recorded_ori_dual[:4]
        self.Panda_right.recorded_gripper = np.c_[self.recorded_gripper_dual[0]].T
        self.Panda_right.recorded_stiffness_lin = self.recorded_stiffness_lin_dual[:3]
        self.Panda_right.recorded_stiffness_ori = self.recorded_stiffness_ori_dual[:3]

        self.Panda_left.recorded_traj = self.recorded_traj_dual[3:]
        self.Panda_left.recorded_ori = self.recorded_ori_dual[4:]
        self.Panda_left.recorded_gripper = np.c_[self.recorded_gripper_dual[1]].T
        self.Panda_left.recorded_stiffness_lin = self.recorded_stiffness_lin_dual[3:]
        self.Panda_left.recorded_stiffness_ori = self.recorded_stiffness_ori_dual[3:]

        self.client.update_configuration({"panda_left_enable_correction":True, "panda_right_enable_correction":True, "dual_enable_correction": True})

        if self.external_record:
            rospy.set_param("/dual_teaching/recording", True)

        self.Panda_left.execute_traj()
        self.Panda_right.execute_traj()
        self.Kinesthetic_Demonstration_BiManual(active=True, verbose=True)

        if self.external_record:
            rospy.set_param("/dual_teaching/recording", False)

        self.client.update_configuration({"panda_left_enable_correction":False, "panda_right_enable_correction":False, "dual_enable_correction": False})
 
    def entanglement_execute_dual(self):
        """
       
        """
        self.Panda_right.execution_traj = self.recorded_traj_dual[:3]
        self.Panda_right.execution_ori = self.recorded_ori_dual[:4]
        self.Panda_right.execution_gripper = np.c_[self.recorded_gripper_dual[0]].T
        self.Panda_right.execution_stiff_lin = self.recorded_stiffness_lin_dual[:3]
        self.Panda_right.execution_stiff_ori = self.recorded_stiffness_ori_dual[:3]

        self.Panda_left.execution_traj = self.recorded_traj_dual[3:]
        self.Panda_left.execution_ori = self.recorded_ori_dual[4:]
        self.Panda_left.execution_gripper = np.c_[self.recorded_gripper_dual[1]].T
        self.Panda_left.execution_stiff_lin = self.recorded_stiffness_lin_dual[3:]
        self.Panda_left.execution_stiff_ori = self.recorded_stiffness_ori_dual[3:]

        self.client.update_configuration({"panda_left_enable_correction":True, "panda_right_enable_correction":True, "dual_enable_correction": True})

        if self.external_record:
            rospy.set_param("/dual_teaching/recording", True)

        self.end=False
        r = rospy.Rate(self.control_freq)
        self.index = 0
        self.Panda_left.index = self.Panda_right.index = self.index

        while 1:
            i_left, attractor_pos_left, attractor_ori_left, stiff_msg_left, beta_left= self.Panda_left.GGP()
                
            i_right, attractor_pos_right, attractor_ori_right, stiff_msg_right, beta_right= self.Panda_right.GGP()
            
            attractor_pos_left = [self.Panda_left.execution_traj[0][i_right], self.Panda_left.execution_traj[1][i_right], self.Panda_left.execution_traj[2][i_right]]
            attractor_ori_left = [self.Panda_left.execution_ori[0][i_right], self.Panda_left.execution_ori[1][i_right], self.Panda_left.execution_ori[2][i_right],self.Panda_left.execution_ori[3][i_right]]
            stiff_msg_left = Float32MultiArray()
            stiff_msg_left.data =beta_left* np.array([self.Panda_left.execution_stiff_lin[0][i_right], self.Panda_left.execution_stiff_lin[1][i_right], self.Panda_left.execution_stiff_lin[2][i_right], self.Panda_left.execution_stiff_ori[0][i_right],self.Panda_left.execution_stiff_ori[1][i_right],self.Panda_left.execution_stiff_ori[2][i_right], 0.0]).astype(np.float32)
            
            attractor_pos_right = [self.Panda_right.execution_traj[0][i_left], self.Panda_right.execution_traj[1][i_left], self.Panda_right.execution_traj[2][i_left]]
            attractor_ori_right = [self.Panda_right.execution_ori[0][i_left], self.Panda_right.execution_ori[1][i_left], self.Panda_right.execution_ori[2][i_left],self.Panda_right.execution_ori[3][i_left]]
            stiff_msg_right = Float32MultiArray()
            stiff_msg_right.data =beta_right *np.array([self.Panda_right.execution_stiff_lin[0][i_left], self.Panda_right.execution_stiff_lin[1][i_left], self.Panda_right.execution_stiff_lin[2][i_left], self.Panda_right.execution_stiff_ori[0][i_left],self.Panda_right.execution_stiff_ori[1][i_left],self.Panda_right.execution_stiff_ori[2][i_left], 0.0]).astype(np.float32)
            
            self.Panda_left.stiffness_pub.publish(stiff_msg_left)
            self.Panda_left.set_attractor(attractor_pos_left, attractor_ori_left)
            self.Panda_left.move_gripper(self.Panda_left.execution_gripper[0, i_right])

            self.Panda_right.stiffness_pub.publish(stiff_msg_right)
            self.Panda_right.set_attractor(attractor_pos_right, attractor_ori_right)
            self.Panda_right.move_gripper(self.Panda_right.execution_gripper[0, i_left])

            if self.end:
                break
            r.sleep() 

        #self.Kinesthetic_Demonstration_BiManual(active=True, verbose=True)

        if self.external_record:
            rospy.set_param("/dual_teaching/recording", False)

        # self.client.update_configuration({"panda_left_enable_correction":False, "panda_right_enable_correction":False, "dual_enable_correction": False})

    def syncronization_execute_dual(self):
        """
       
        """
        self.Panda_right.execution_traj = self.recorded_traj_dual[:3]
        self.Panda_right.execution_ori = self.recorded_ori_dual[:4]
        self.Panda_right.execution_gripper = np.c_[self.recorded_gripper_dual[0]].T
        self.Panda_right.execution_stiff_lin = self.recorded_stiffness_lin_dual[:3]
        self.Panda_right.execution_stiff_ori = self.recorded_stiffness_ori_dual[:3]

        self.Panda_left.execution_traj = self.recorded_traj_dual[3:]
        self.Panda_left.execution_ori = self.recorded_ori_dual[4:]
        self.Panda_left.execution_gripper = np.c_[self.recorded_gripper_dual[1]].T
  
        self.Panda_left.execution_stiff_lin = self.recorded_stiffness_lin_dual[3:]
        self.Panda_left.execution_stiff_ori = self.recorded_stiffness_ori_dual[3:]

        self.client.update_configuration({"panda_left_enable_correction":True, "panda_right_enable_correction":True, "dual_enable_correction": True})

        if self.external_record:
            rospy.set_param("/dual_teaching/recording", True)

        self.end=False
        r = rospy.Rate(self.control_freq)
        self.index = 0
        self.Panda_left.index = self.Panda_right.index = self.index
        while 1:
            i_left, attractor_pos_left, attractor_ori_left, stiff_msg_left, beta_left= self.Panda_left.GGP()
                
            i_right, attractor_pos_right, attractor_ori_right, stiff_msg_right, beta_right= self.Panda_right.GGP()
            
            if np.abs(i_left-i_right)>6:
                max_index= np.max([i_left, i_right])-4
                i_left=np.min([i_left, max_index])
                i_right=np.min([i_right, max_index])
                self.Panda_left.index=i_left
                self.Panda_right.index=i_right

            attractor_pos_left = [self.Panda_left.execution_traj[0][i_left], self.Panda_left.execution_traj[1][i_left], self.Panda_left.execution_traj[2][i_left]]
            attractor_ori_left = [self.Panda_left.execution_ori[0][i_left], self.Panda_left.execution_ori[1][i_left], self.Panda_left.execution_ori[2][i_left],self.Panda_left.execution_ori[3][i_left]]
            stiff_msg_left = Float32MultiArray()
            stiff_msg_left.data =beta_left* np.array([self.Panda_left.execution_stiff_lin[0][i_left], self.Panda_left.execution_stiff_lin[1][i_left], self.Panda_left.execution_stiff_lin[2][i_left], self.Panda_left.execution_stiff_ori[0][i_left],self.Panda_left.execution_stiff_ori[1][i_left],self.Panda_left.execution_stiff_ori[2][i_left], 0.0]).astype(np.float32)
            
            attractor_pos_right = [self.Panda_right.execution_traj[0][i_right], self.Panda_right.execution_traj[1][i_right], self.Panda_right.execution_traj[2][i_right]]
            attractor_ori_right = [self.Panda_right.execution_ori[0][i_right], self.Panda_right.execution_ori[1][i_right], self.Panda_right.execution_ori[2][i_right],self.Panda_right.execution_ori[3][i_right]]
            stiff_msg_right = Float32MultiArray()
            stiff_msg_right.data =beta_right *np.array([self.Panda_right.execution_stiff_lin[0][i_right], self.Panda_right.execution_stiff_lin[1][i_right], self.Panda_right.execution_stiff_lin[2][i_right], self.Panda_right.execution_stiff_ori[0][i_right],self.Panda_right.execution_stiff_ori[1][i_right],self.Panda_right.execution_stiff_ori[2][i_right], 0.0]).astype(np.float32)
            
            self.Panda_left.stiffness_pub.publish(stiff_msg_left)
            self.Panda_left.set_attractor(attractor_pos_left, attractor_ori_left)
            self.Panda_left.move_gripper(self.Panda_left.execution_gripper[0, i_left])

            self.Panda_right.stiffness_pub.publish(stiff_msg_right)
            self.Panda_right.set_attractor(attractor_pos_right, attractor_ori_right)
            self.Panda_right.move_gripper(self.Panda_right.execution_gripper[0, i_right])

            if self.end:
                break
            r.sleep() 

        if self.external_record:
            rospy.set_param("/dual_teaching/recording", False)    

    def go_to_start(self):
        """
        Function used to set both arms to the first position of the demonstrated trajectories.
        """
        goal = PoseStamped()
        goal.header.seq = 1
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"
        goal.pose.position.x = self.recorded_traj_dual[0][0]
        goal.pose.position.y = self.recorded_traj_dual[1][0]
        goal.pose.position.z = self.recorded_traj_dual[2][0]

        goal.pose.orientation.w = self.recorded_ori_dual[0][0]
        goal.pose.orientation.x = self.recorded_ori_dual[1][0]
        goal.pose.orientation.y = self.recorded_ori_dual[2][0]
        goal.pose.orientation.z = self.recorded_ori_dual[3][0]

        self.Panda_right.goto_pub.publish(goal)

        goal = PoseStamped()
        goal.header.seq = 1
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"
        goal.pose.position.x = self.recorded_traj_dual[3][0]
        goal.pose.position.y = self.recorded_traj_dual[4][0]
        goal.pose.position.z = self.recorded_traj_dual[5][0]

        goal.pose.orientation.w = self.recorded_ori_dual[4][0]
        goal.pose.orientation.x = self.recorded_ori_dual[5][0]
        goal.pose.orientation.y = self.recorded_ori_dual[6][0]
        goal.pose.orientation.z = self.recorded_ori_dual[7][0]

        self.Panda_left.goto_pub.publish(goal)


    def save(self, data='last'):
        """
        Function to save the last bimanual demonstration to a file.
        """
        np.savez(str(pathlib.Path().resolve()) + '/data/' + 'dual_' + str(data) + '.npz',
                 recorded_traj_dual=self.recorded_traj_dual,
                 recorded_ori_dual=self.recorded_ori_dual,
                 recorded_gripper_dual=self.recorded_gripper_dual,
                 recorded_stiffness_lin_dual =self.recorded_stiffness_lin_dual,
                 recorded_stiffness_ori_dual = self.recorded_stiffness_ori_dual)

    def load(self, data='last'):
        """
        Function to load a previously saved bimanual demonstration.
        """
        data = np.load(str(pathlib.Path().resolve()) + '/data/' + 'dual_' + str(data) + '.npz')

        self.recorded_traj_dual = data['recorded_traj_dual'],
        self.recorded_ori_dual = data['recorded_ori_dual'],
        self.recorded_gripper_dual = data['recorded_gripper_dual'],
        self.recorded_stiffness_lin_dual = data['recorded_stiffness_lin_dual'],
        self.recorded_stiffness_ori_dual = data['recorded_stiffness_ori_dual'],
        self.recorded_traj_dual = self.recorded_traj_dual[0]
        self.recorded_ori_dual = self.recorded_ori_dual[0]
        self.recorded_gripper_dual = self.recorded_gripper_dual[0]
        self.recorded_stiffness_lin_dual = self.recorded_stiffness_lin_dual[0]
        self.recorded_stiffness_ori_dual = self.recorded_stiffness_ori_dual[0]

    def continue_dual_traj(self, index, traj):
        """
        Function that is used to verify if either arm is outside the attractor distance threshold.
        :param index: The index of the location on the trajectory to verify
        :param traj: The trajectory to compare against
        """
        if not (self.Panda_left.continue_traj(index, traj[3:])) or not (
                self.Panda_right.continue_traj(index, traj[:3])):
            return False
        else:
            return True
