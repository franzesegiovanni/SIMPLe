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
    #ILoSA.connect_ROS()
    time.sleep(1)
    BiManualTeaching.Panda_left.home_gripper()
    BiManualTeaching.Panda_right.home_gripper()
    time.sleep(1)
# %%
BiManualTeaching.Panda_left.home()
BiManualTeaching.Panda_right.home()
# %%
BiManualTeaching.Kinesthetic_Demonstration_BiManual()
# %%
BiManualTeaching.Panda_left.home()
BiManualTeaching.Panda_right.home()
#%%
BiManualTeaching.execute_dual()
# %%
BiManualTeaching.Panda_left.home()
BiManualTeaching.Panda_right.home()
#%%
BiManualTeaching.execute_dual()
# %%
BiManualTeaching.Panda_left.home()
BiManualTeaching.Panda_right.home()
#%%
BiManualTeaching.entanglement_execute_dual()
# %%
BiManualTeaching.Panda_left.home()
BiManualTeaching.Panda_right.home()
#%%
BiManualTeaching.entanglement_execute_dual()
# %%
BiManualTeaching.Panda_left.home()
BiManualTeaching.Panda_right.home()
#%%
BiManualTeaching.syncronization_execute_dual()
# %%
BiManualTeaching.Panda_left.home()
BiManualTeaching.Panda_right.home()
#%%
BiManualTeaching.syncronization_execute_dual()
#%%