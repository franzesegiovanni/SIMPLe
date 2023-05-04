"""
Authors: Giovanni Franzese 
Email: g.franzese@tudelft.nl
Cognitive Robotics, TU Delft
This code is part of TERI (TEaching Robots Interactively) project
"""
#%%
import matplotlib
from dual_panda_ILoSA import DualPanda
import time
import rospy
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import numpy as np
#%%
if __name__ == '__main__':
    BiManualTeaching=DualPanda()
    BiManualTeaching.Panda_right.enable_corr=True
    BiManualTeaching.Panda_left.enable_corr=True
    time.sleep(1)
    BiManualTeaching.Panda_right.home()
    #%%
    BiManualTeaching.Panda_right.Kinesthetic_Demonstration()
    #%%
    BiManualTeaching.Panda_right.home()   
    # %%  Execute the Motion and see what it learned
    BiManualTeaching.Panda_right.execute_traj()
    # %%
    BiManualTeaching.Panda_right.end=True
    BiManualTeaching.Panda_right.home()
    #%% HERE YOU CAN GIVE KINESTHETIC CORRECTIONS
    BiManualTeaching.Panda_right.Kinesthetic_Demonstration(active=True)
    BiManualTeaching.Panda_right.execute_traj()  
    #%%
    BiManualTeaching.Panda_right.end=True
    BiManualTeaching.Panda_right.home()
    #%% FORGET ABOUT THE CORRECTIONS
    BiManualTeaching.Panda_right.execution_traj = np.asarray(BiManualTeaching.Panda_right.recorded_traj)
    BiManualTeaching.Panda_right.execution_ori = np.asarray(BiManualTeaching.Panda_right.recorded_ori)
    #%% HERE YOU GIVE CORRECTIONS SPREADING THE FEEDBACK ON THE DATABASE LIKE ILOSA
    BiManualTeaching.Panda_right.correction_mode=0
    BiManualTeaching.Panda_right.execute_traj()
    BiManualTeaching.Panda_right.Kinesthetic_Demonstration(active=True)
    #%% FORGET ABOUT THE CORRECTIONS
    BiManualTeaching.Panda_right.execution_traj = np.asarray(BiManualTeaching.Panda_right.recorded_traj)
    BiManualTeaching.Panda_right.execution_ori = np.asarray(BiManualTeaching.Panda_right.recorded_ori)
    #%% HERE YOU GIVE CORRECTIONS TAKING CONTROL LIKE HG DAGGER
    BiManualTeaching.Panda_right.correction_mode=1
    BiManualTeaching.Panda_right.execute_traj()
    BiManualTeaching.Panda_right.Kinesthetic_Demonstration(active=True)