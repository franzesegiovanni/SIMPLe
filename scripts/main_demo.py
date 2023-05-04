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
#%%
if __name__ == '__main__':
    BiManualTeaching=DualPanda()
    time.sleep(1)
    BiManualTeaching.Panda_left.home()
    BiManualTeaching.Panda_right.home()

    #%%
    BiManualTeaching.Panda_left.Kinesthetic_Demonstration()

    # %%
    BiManualTeaching.Panda_left.home()
    #%%
    BiManualTeaching.Panda_right.Kinesthetic_Demonstration()
    #%%
    BiManualTeaching.Panda_right.home()   
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
    # %%
    BiManualTeaching.correction_execute_dual()
    # %%
    BiManualTeaching.Panda_left.home()
    BiManualTeaching.Panda_right.home()
    #%%    TEST HOW IT PERFORMS IN THE BIMANUAL TASK
    BiManualTeaching.execute_dual()
    # %%
    BiManualTeaching.Panda_left.home()
    BiManualTeaching.Panda_right.home()
    #%%    TEST HOW IT PERFORMS IN THE BIMANUAL TASK
    BiManualTeaching.entanglement_execute_dual()
    #%% ASK TWO STUDENDTS TO GIVE A DEMO TOGETHER
    BiManualTeaching.Panda_left.home()
    BiManualTeaching.Panda_right.home()
    #%%
    BiManualTeaching.execute_dual()
    
    #%% ASK A STUDENT TO GIVE DEMONSTRATIONS ALONE ONE ARM AT THE TIME
    BiManualTeaching.Panda_left.Kinesthetic_Demonstration()
    # %%
    BiManualTeaching.Panda_left.home()
    #%%
    BiManualTeaching.Panda_right.Kinesthetic_Demonstration()
    #%%
    BiManualTeaching.Panda_right.home()   
    #%% Syncronization
    BiManualTeaching.Panda_right.execute_traj()
    BiManualTeaching.Panda_left.execute_traj()
    BiManualTeaching.Kinesthetic_Demonstration_BiManual(active=True)
    # %%
    BiManualTeaching.Panda_left.home()
    BiManualTeaching.Panda_right.home()
    #%%   
    BiManualTeaching.execute_dual()


     #%% ASK A STUDENT TO GIVE DEMONSTRATIONS ALONE ONE ARM AT THE TIME
    BiManualTeaching.Panda_left.home()
    BiManualTeaching.Panda_right.home()

    #%%
    BiManualTeaching.Kinesthetic_Demonstration_BiManual()
    #%%
    BiManualTeaching.Panda_left.home()
    BiManualTeaching.Panda_right.home()
    #%%  
    BiManualTeaching.execute_dual()


    #%% ASK A STUDENT TO GIVE DEMONSTRATIONS ONE FOR EACH ARM
    BiManualTeaching.Panda_left.home()
    BiManualTeaching.Panda_right.home()

    #%%
    BiManualTeaching.Kinesthetic_Demonstration_BiManual()
    #%%
    BiManualTeaching.Panda_left.home()
    BiManualTeaching.Panda_right.home()
    #%%  
    BiManualTeaching.execute_dual()

    #%%
    BiManualTeaching.correction_execute_dual()
    #%%
    BiManualTeaching.Panda_left.home()
    BiManualTeaching.Panda_right.home()

# %%
