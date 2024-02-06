"""
Authors: Giovanni Franzese 
Email: g.franzese@tudelft.nl
Cognitive Robotics, TU Delft
This code is part of TERI (TEaching Robots Interactively) project
"""
#%%
from  SIMPLe_bimanual.dual_panda import DualPanda
from pynput.keyboard import Listener, Key
import time
import rospy
import numpy as np

class Coordination(DualPanda):
    def __init__(self):
        super().__init__()

        self.listener_arrow = Listener(on_press=self._on_press_arrow, interval=1/self.control_freq)

        self.listener_arrow.start()

        self.look_ahead=30 # this how many steps in the future the robot is going to move after one input on the keyboard
    def _on_press_arrow(self, key):
        # This function runs on the background and checks if a keyboard key was pressed
        if key == Key.right:
            self.right = True
            self.index_right=int(self.index_right+self.look_ahead)
        if key == Key.left:
            self.right = True
            self.index_left=int(self.index_left+self.look_ahead)
            

    def syncronize(self):
        
        self.index_right=int(0)
        self.index_left=int(0)

        ind_left=0
        ind_right=0
        self.end=False
        r = rospy.Rate(self.control_freq)

        print("Press Esc to stop controlling the robot")


        attractor_pos_right = [self.Panda_right.recorded_traj[0][self.index_right],  self.Panda_right.recorded_traj[1][self.index_right],  self.Panda_right.recorded_traj[2][self.index_right]]
        attractor_pos_left = [self.Panda_left.recorded_traj[0][self.index_left],  self.Panda_left.recorded_traj[1][self.index_left],  self.Panda_left.recorded_traj[2][self.index_left]]

        if np.linalg.norm(np.array(attractor_pos_right)-self.Panda_right.cart_pos) > 0.05 or np.linalg.norm(np.array(attractor_pos_left)-self.Panda_left.cart_pos) > 0.05:
            print("Robots are too far away from the starting position, send them to start first")
            self.end=True
            return

        while not self.end:
            ind_right=np.min([ind_right+np.clip(self.index_right-ind_right,0,1), self.Panda_right.recorded_traj.shape[1]-1])
            ind_left=np.min([ind_left+np.clip(self.index_left-ind_left,0,1), self.Panda_left.recorded_traj.shape[1]-1])
            attractor_pos_right = [self.Panda_right.recorded_traj[0][ind_right],  self.Panda_right.recorded_traj[1][ind_right],  self.Panda_right.recorded_traj[2][ind_right]]
            attractor_ori_right = [ self.Panda_right.recorded_ori[0][ind_right],  self.Panda_right.recorded_ori[1][ind_right],   self.Panda_right.recorded_ori[2][ind_right],  self.Panda_right.recorded_ori[3][ind_right]]

            attractor_pos_left = [self.Panda_left.recorded_traj[0][ind_left],  self.Panda_left.recorded_traj[1][ind_left],  self.Panda_left.recorded_traj[2][ind_left]]
            attractor_ori_left = [ self.Panda_left.recorded_ori[0][ind_left],  self.Panda_left.recorded_ori[1][ind_left],   self.Panda_left.recorded_ori[2][ind_left],  self.Panda_left.recorded_ori[3][ind_left]]

            self.Panda_right.set_attractor(attractor_pos_right, attractor_ori_right)
            self.Panda_right.move_gripper(self.Panda_right.recorded_gripper[0, ind_right])

            self.Panda_left.set_attractor(attractor_pos_left, attractor_ori_left)
            self.Panda_left.move_gripper(self.Panda_left.recorded_gripper[0, ind_left])


            r.sleep()
        


#%%
if __name__ == '__main__':
    BiManualTeaching=Coordination()
    time.sleep(1)
    #%%
    BiManualTeaching.Panda_right.load()
    BiManualTeaching.Panda_left.load()  
    #%%
    BiManualTeaching.Panda_right.go_to_start()
    BiManualTeaching.Panda_left.go_to_start()  
    #%%
    BiManualTeaching.Panda_left.home()
    BiManualTeaching.Panda_right.home()
    #%%
    BiManualTeaching.Panda_left.home_gripper()
    BiManualTeaching.Panda_right.home_gripper()
    #%%
    BiManualTeaching.Panda_left.Kinesthetic_Demonstration()

    # %%
    BiManualTeaching.Panda_left.home()
    #%%
    BiManualTeaching.Panda_right.Kinesthetic_Demonstration()
    #%%
    BiManualTeaching.Panda_right.home()
    # %%
    BiManualTeaching.Panda_left.save() 
    BiManualTeaching.Panda_right.save()      

    #%%
    BiManualTeaching.syncronize()
    # %%
  