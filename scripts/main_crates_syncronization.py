"""
Authors: Giovanni Franzese 
Email: g.franzese@tudelft.nl
Cognitive Robotics, TU Delft
This code is part of TERI (TEaching Robots Interactively) project
"""
#%%
import matplotlib
from  dual_panda import DualPanda
import time
import rospy
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
#%%
if __name__ == '__main__':
    BiManualTeaching=DualPanda()
#time.sleep(1)
# BiManualTeaching.Panda_left.home_gripper()
# BiManualTeaching.Panda_right.home_gripper()
    time.sleep(1)
    #%%
    BiManualTeaching.Panda_right.load()
    BiManualTeaching.Panda_left.load()  
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
    # %%  EXECUTE DEMONSTRATION AT THE SAME TIME
    BiManualTeaching.Panda_right.execute_traj()
    BiManualTeaching.Panda_left.execute_traj()
    # %%
    BiManualTeaching.Panda_left.home()
    BiManualTeaching.Panda_right.home()
    # EXECUTE DEMONSTRATION AT THE SAME TIME BUT GIVE CORRECTIONS
    # %%
    BiManualTeaching.Panda_right.execute_traj()
    BiManualTeaching.Panda_left.execute_traj()
    BiManualTeaching.Kinesthetic_Demonstration_BiManual(active=True)
    # %%
    BiManualTeaching.Panda_left.home()
    BiManualTeaching.Panda_right.home()
    #%%    TEST HOW IT PERFORMS IN THE BIMANUAL TASK
    BiManualTeaching.execute_dual()
#%%
    BiManualTeaching.Panda_left.home()
    BiManualTeaching.Panda_right.home()
    #%%    TEST HOW IT PERFORMS IN THE BIMANUAL TASK
    rospy.set_param("/dual_teaching/execution_factor", 2)
    BiManualTeaching.execute_dual()
    #%%
    BiManualTeaching.Panda_left.home()
    BiManualTeaching.Panda_right.home()
    #%%    TEST HOW IT PERFORMS IN THE BIMANUAL TASK
    rospy.set_param("/dual_teaching/execution_factor", 3)
    BiManualTeaching.execute_dual()
    #%%
    BiManualTeaching.Panda_left.home()
    BiManualTeaching.Panda_right.home()
    #%%    TEST HOW IT PERFORMS IN THE BIMANUAL TASK
    rospy.set_param("/dual_teaching/execution_factor", 4)
    BiManualTeaching.execute_dual()
    #%%
    BiManualTeaching.save()