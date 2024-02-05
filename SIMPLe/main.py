"""
Authors: Giovanni Franzese
Email: g.franzese@tudelft.nl
Cognitive Robotics, TU Delft
This code is part of TERI (TEaching Robots Interactively) project
"""
#%%
from SIMPLe import SIMPLe
import time
from geometry_msgs.msg import PoseStamped
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
    time.sleep(1)
    print("Save the data") 
    SIMPLe.save()
    #%%
    time.sleep(1)
    print("Load the data") 
    SIMPLe.load()    
    #%%
    time.sleep(1)
    print("Reset to the starting cartesian position")
    start = PoseStamped()

    start.pose.position.x = SIMPLe.training_traj[0,0]
    start.pose.position.y = SIMPLe.training_traj[0,1]
    start.pose.position.z = SIMPLe.training_traj[0,2]
    
    start.pose.orientation.w = SIMPLe.training_ori[0,0] 
    start.pose.orientation.x = SIMPLe.training_ori[0,1] 
    start.pose.orientation.y = SIMPLe.training_ori[0,2] 
    start.pose.orientation.z = SIMPLe.training_ori[0,3] 
    SIMPLe.go_to_pose(start)
    
    #%% 
    time.sleep(1)
    print("Interactive Control Starting")
    SIMPLe.control()

# %%
