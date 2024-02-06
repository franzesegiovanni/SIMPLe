"""
Authors: Giovanni Franzese, June 2022
Email: g.franzese@tudelft.nl
Cognitive Robotics, TU Delft
This code is part of TERI (TEaching Robots Interactively) project
"""
#%%
# !/usr/bin/env python
import rospy
import numpy as np
import quaternion
from geometry_msgs.msg import  PoseStamped
import pathlib
from pynput.keyboard import Listener, Key
from .panda import Panda

class DualPanda:
    def __init__(self, arm_id_right='panda_right', arm_id_left='panda_left'):
        rospy.init_node('DualArmControl', anonymous=True)
  
        self.control_freq = 30

        # Factors that multiply the feedback from the Spacemouse to allow for more rough or granular control
        self.feedback_factor_pos = 0.02
        self.feedback_factor_ori = 0.02
        self.feedback_factor_stiffness = 0.05

        self.target_coupling_stiffness = 200

        self.Panda_right = Panda( self.control_freq, arm_id_right)
        self.Panda_left = Panda( self.control_freq, arm_id_left)
        
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
        if key == Key.esc:
            self.end = True
            self.Panda_left.end = True
            self.Panda_right.end = True

    def Kinesthetic_Demonstration_BiManual(self, active=False):
        """
        Funtion to record a bimanual demonstration.

        :param active: A boolean
        that determines the stiffness of the arms. If you have Active equal to true, you are recording but the
        manipulators' stiffness is not dropped to zero. You can for example learn to synchronize what two
        manipulators are doing
        """

        r = rospy.Rate(self.control_freq)
        
        if active is False:
            self.Panda_left.Passive()
            self.Panda_right.Passive()
        else:
            self.Panda_left.Active()
            self.Panda_right.Active()

        self.end = False

        print("Recording started. Press e to stop.")
        self.recorded_traj_dual = np.r_[self.Panda_right.cart_pos, self.Panda_left.cart_pos]
        self.recorded_joint_dual = np.r_[self.Panda_right.joint_pos, self.Panda_left.joint_pos]
        self.recorded_ori_dual = np.r_[self.Panda_right.cart_ori, self.Panda_left.cart_ori]
        self.recorded_gripper_dual = np.array([self.Panda_right.gripper_width, self.Panda_left.gripper_width])
        self.recorded_stiffness_lin_dual = np.array([self.Panda_right.K_cart, self.Panda_right.K_cart, self.Panda_right.K_cart, self.Panda_left.K_cart, self.Panda_left.K_cart, self.Panda_left.K_cart])
        self.recorded_stiffness_ori_dual = np.array([self.Panda_right.K_ori, self.Panda_right.K_ori, self.Panda_right.K_ori, self.Panda_left.K_ori, self.Panda_left.K_ori, self.Panda_left.K_ori])
        self.end = False
        while (not self.end):
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

    def execute_dual(self):
        """
        Function to execute a bimanual demonstration. This execution will pause if one of the arms is outside their
        respective attractor distance threshold
        """
        r = rospy.Rate(self.control_freq)

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

        # Code to calculate the difference between the arms and publish it to the coupling topic
        d_coupling = np.array(position_right) - np.array(position_left)
        diff_msg = PoseStamped()
        diff_msg.header.seq = 1
        diff_msg.header.stamp = rospy.Time.now()
        diff_msg.pose.position.x = d_coupling[0]
        diff_msg.pose.position.y = d_coupling[1]
        diff_msg.pose.position.z = d_coupling[2]        

        self.coupling_diff_pub.publish(diff_msg)

        self.Panda_left.set_K.update_configuration({"coupling_translational_stiffness": self.target_coupling_stiffness})


        r = rospy.Rate(self.control_freq)

        while self.index < self.recorded_traj_dual.shape[1]:


            if self.continue_dual_traj(self.index, self.recorded_traj_dual):

                position_right = [self.Panda_right.execution_traj[0][self.index], self.Panda_right.execution_traj[1][self.index], self.Panda_right.execution_traj[2][self.index]]
                orientation_right = [self.Panda_right.execution_ori[0][self.index], self.Panda_right.execution_ori[1][self.index], self.Panda_right.execution_ori[2][self.index],
                           self.Panda_right.execution_ori[3][self.index]]


                position_left = [self.Panda_left.execution_traj[0][self.index], self.Panda_left.execution_traj[1][self.index], self.Panda_left.execution_traj[2][self.index]]
                orientation_left = [self.Panda_left.execution_ori[0][self.index], self.Panda_left.execution_ori[1][self.index], self.Panda_left.execution_ori[2][self.index],
                               self.Panda_left.execution_ori[3][self.index]]


                self.Panda_right.set_attractor(position_right, orientation_right)
                self.Panda_left.set_attractor(position_left, orientation_left)

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

        self.Panda_left.execute_traj()
        self.Panda_right.execute_traj()
        self.Kinesthetic_Demonstration_BiManual(active=True)

        self.client.update_configuration({"panda_left_enable_correction":False, "panda_right_enable_correction":False, "dual_enable_correction": False})
 
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
