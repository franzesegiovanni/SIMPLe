"""
Authors: Giovanni Franzese 
Email: g.franzese@tudelft.nl
Cognitive Robotics, TU Delft
This code is part of TERI (TEaching Robots Interactively) project
"""
#%%
import matplotlib
from SIMPLe_bimanual.dual_panda import DualPanda
import time

from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
#%%
if __name__ == '__main__':
    BiManualTeaching=DualPanda()
# #time.sleep(1)
    BiManualTeaching.Panda_left.home_gripper()
    BiManualTeaching.Panda_right.home_gripper()
    time.sleep(1)
    #%%
    BiManualTeaching.Panda_left.move_gripper(0.0)
    BiManualTeaching.Panda_right.move_gripper(0.0)
    #%%
    BiManualTeaching.Kinesthetic_Demonstration_BiManual()
   # %%
    BiManualTeaching.go_to_start()
    # EXECUTE DEMONSTRATION AT THE SAME TIME BUT GIVE CORRECTIONS
    #%%
    BiManualTeaching.execute_dual()
    #%%
    BiManualTeaching.go_to_start()
    #%% Give perturbations
    BiManualTeaching.execute_dual() 
     #%%
    BiManualTeaching.go_to_start()

# %%
