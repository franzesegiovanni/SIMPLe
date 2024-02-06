"""
Authors: Giovanni Franzese 
Email: g.franzese@tudelft.nl
Cognitive Robotics, TU Delft
This code is part of TERI (TEaching Robots Interactively) project
"""
#%%
from SIMPLe_bimanual.dual_panda import DualPanda
import time
#%%
if __name__ == '__main__':
    BiManualTeaching=DualPanda()
    time.sleep(1)
    # BiManualTeaching.Panda_left.home_gripper()
    # BiManualTeaching.Panda_right.home_gripper()
    #%%
    BiManualTeaching.Kinesthetic_Demonstration_BiManual()
    #%%
    BiManualTeaching.save(data='conducting')
    #%%
    BiManualTeaching.load(data='conducting')
    # %%
    BiManualTeaching.go_to_start()
    # %% Close
    BiManualTeaching.Panda_left.move_gripper(0.0)
    BiManualTeaching.Panda_right.move_gripper(0.0)

    # %% Open
    BiManualTeaching.Panda_left.move_gripper(1.0)
    BiManualTeaching.Panda_right.move_gripper(1.0)
    #%%
    BiManualTeaching.Panda_left.set_K.update_configuration({"coupling_translational_stiffness": 0})

    BiManualTeaching.Kinesthetic_Demonstration_BiManual()
    # %%
    BiManualTeaching.go_to_start()
    #%%
    BiManualTeaching.execute_dual()
    # %%
    BiManualTeaching.save(data='conductor_linda')
    #%%
    BiManualTeaching.load(data='conductor_linda')
    # %%
    BiManualTeaching.Panda_left.home_gripper()
    BiManualTeaching.Panda_right.home_gripper()
# %%
