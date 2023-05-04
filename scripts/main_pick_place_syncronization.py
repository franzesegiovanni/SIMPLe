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

from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
#%%
if __name__ == '__main__':
    BiManualTeaching=DualPanda()
    time.sleep(1)
    BiManualTeaching.Panda_left.home_gripper()
    time.sleep(1)
    BiManualTeaching.Panda_right.home_gripper()
    time.sleep(1)
    # %%
    BiManualTeaching.Panda_left.home()
    BiManualTeaching.Panda_right.home() 
    #%%
    BiManualTeaching.Panda_right.Kinesthetic_Demonstration()
    # %%
    BiManualTeaching.Panda_right.home()
    BiManualTeaching.Panda_right.home_gripper()

    #%%
    BiManualTeaching.Panda_left.Kinesthetic_Demonstration()
    # %%
    BiManualTeaching.Panda_left.home()
    BiManualTeaching.Panda_left.home_gripper()
    #%%
    # %%
    BiManualTeaching.Panda_right.execute_traj()
    BiManualTeaching.Panda_left.execute_traj()
    BiManualTeaching.Kinesthetic_Demonstration_BiManual(active=True)
    #%%
    BiManualTeaching.Panda_left.home()
    BiManualTeaching.Panda_left.home_gripper()
    BiManualTeaching.Panda_right.home()
    BiManualTeaching.Panda_right.home_gripper()
# %%
BiManualTeaching.execute_dual()
# %%
