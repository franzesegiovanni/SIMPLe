"""
Authors: Giovanni Franzese
Email: g.franzese@tudelft.nl
Cognitive Robotics, TU Delft
This code is part of TERI (TEaching Robots Interactively) project
"""
#%%
from SIMPLe import SIMPLe
import time
import rospy
#%%
if __name__ == '__main__':
    rospy.init_node('SIMPLe', anonymous=True)
    SIMPLe=SIMPLe()
    SIMPLe.connect_ROS()
    time.sleep(1)
    # SIMPLe.home_gripper()
    #%%
    time.sleep(1)
    print("Record of the cartesian trajectory")
    SIMPLe.Record_Demonstration()     
    #%%
    SIMPLe.go_to_start()
    #%%
    time.sleep(1)
    print("Save the data") 
    SIMPLe.save()
    #%%
    time.sleep(1)
    print("Load the data") 
    SIMPLe.load()    
    #%% 
    time.sleep(1)
    print("Interactive Control Starting")
    SIMPLe.control()

# %%
