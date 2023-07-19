"""
Authors: Giovanni Franzese and Ravi Prakash
Email: g.franzese@tudelft.nl
Cognitive Robotics, TU Delft
This code is part of TERI (TEaching Robots Interactively) project

This is the code used for the experiment of reshalving 
"""
#%%
from modules import GILoSA_tag
import time
from geometry_msgs.msg import PoseStamped
import rospy
from sklearn.gaussian_process.kernels import RBF, Matern,WhiteKernel, ConstantKernel as C
import numpy as np
import pickle
from copy import copy
#%%
if __name__ == '__main__':
    rospy.init_node('GILoSA', anonymous=True)
    GILoSA=GILoSA_tag()
    GILoSA.connect_ROS()

    GILoSA.null_stiff=[0]
    time.sleep(1)

  
    # pnp view
    GILoSA.view_marker = PoseStamped()
    GILoSA.view_marker.header.frame_id = "panda_link0"
    GILoSA.view_marker.pose.position.x = 0.306
    GILoSA.view_marker.pose.position.y = 0.408
    GILoSA.view_marker.pose.position.z = 0.4435
    GILoSA.view_marker.pose.orientation.w = 0.53
    GILoSA.view_marker.pose.orientation.x = 0.845
    GILoSA.view_marker.pose.orientation.y = 0.0054
    GILoSA.view_marker.pose.orientation.z = 0.073

    GILoSA.home_gripper()



    #%%
 
    # GILoSA.go_to_pose(GILoSA.view_marker)
    GILoSA.record_source_distribution()
    f = open("results/pnp/source.pkl","wb")
    # write the python object (dict) to pickle file
    pickle.dump(GILoSA.source_distribution,f)
    # close file
    f.close()  

    #%%
    GILoSA.record_source_distribution()
    #%%
    GILoSA.Record_traj_tags()

    #%%
    GILoSA.save_traj_tags()

    #%%
    GILoSA.load_traj_tags()
    #%%
    GILoSA.source_distribution=[]
    GILoSA.target_distribution=[]
    #GILoSA.source_distribution=GILoSA.Record_tags_goto(GILoSA.source_distribution)
    GILoSA.source_distribution=GILoSA.Record_tags(GILoSA.source_distribution)

    print("Source len",len(GILoSA.source_distribution))
    print("Save the  source distribution data") 
    GILoSA.save_distributions()  
    #%%
    time.sleep(1)
    print("Record of the cartesian trajectory")
    GILoSA.Record_Demonstration()  
    GILoSA.save()
    #%%
    # Save traj configuration
    f = open("results/pnp/traj_demo.pkl","wb")
    # write the python object (dict) to pickle file
    pickle.dump(GILoSA.training_traj,f)
    # close file
    f.close()
   
    f = open("results/pnp/traj_demo_ori.pkl","wb")
    # write the python object (dict) to pickle file
    pickle.dump(GILoSA.training_ori,f)
    # close file
    f.close()

    #%%
    i=0

    #%%
    GILoSA.load_distributions()     #load source 
    GILoSA.load()
    GILoSA.load_traj_tags()
    GILoSA.home_gripper()
    #%%
    GILoSA.target_distribution=[]
    # GILoSA.target_distribution=GILoSA.Record_tags_goto(GILoSA.target_distribution)
    GILoSA.target_distribution=GILoSA.Record_tags(GILoSA.target_distribution)

    print("Target len", len(GILoSA.target_distribution) )
    # Save start configuration

    #%%
    # Save goal configuration
    f = open("results/pnp/target_"+str(i)+".pkl","wb")
    # write the python object (dict) to pickle file
    pickle.dump(GILoSA.target_distribution,f)
    # close file
    f.close()   
    #%%
    if type(GILoSA.target_distribution) != type(GILoSA.source_distribution):
        raise TypeError("Both the distribution must be a numpy array.")
    elif not(isinstance(GILoSA.target_distribution, np.ndarray)) and not(isinstance(GILoSA.source_distribution, np.ndarray)):
        GILoSA.convert_distribution_to_array(use_orientation=True) #this needs to be a function of every sensor class
        # if you want to use the orientation, you can use
        #GILoSA.convert_distribution_to_array(use_orientation=True) #this needs to be a function of every sensor class

    #%%
    time.sleep(1)
    print("Find the transported policy")
    # GILoSA.kernel_transport=C(0.1,[0.1,0.1]) * RBF(length_scale=[0.3], length_scale_bounds=[0.1,1]) + WhiteKernel(0.000001, [0.000001,0.000001])
    GILoSA.kernel_transport=C(0.1) * RBF(length_scale=[0.1],  length_scale_bounds=[0.1,0.5]) + WhiteKernel(0.0000001, [0.0000001,0.0000001])

    GILoSA.fit_transportation()
    GILoSA.apply_transportation()
    #%%
    # Save traj configuration
    f = open("results/pnp/traj_"+str(i)+".pkl","wb")
    # write the python object (dict) to pickle file
    pickle.dump(GILoSA.training_traj,f)
    # close file
    f.close()


    f = open("results/pnp/traj_ori_"+str(i)+".pkl","wb")
    # write the python object (dict) to pickle file
    pickle.dump(GILoSA.training_ori,f)
    # close file
    f.close()

    #%%
    time.sleep(1)
    print("Reset to the starting cartesian position")
    start = PoseStamped()

    start.pose.position.x = GILoSA.training_traj[0,0]
    start.pose.position.y = GILoSA.training_traj[0,1]
    start.pose.position.z = GILoSA.training_traj[0,2]
    
    start.pose.orientation.w = GILoSA.training_ori[0,0] 
    start.pose.orientation.x = GILoSA.training_ori[0,1] 
    start.pose.orientation.y = GILoSA.training_ori[0,2] 
    start.pose.orientation.z = GILoSA.training_ori[0,3] 
    GILoSA.go_to_pose(start)
    GILoSA.index=0
    
    #%% 
    time.sleep(1)
    print("Interactive Control Starting")
    GILoSA.mu_index=0
    GILoSA.Interactive_Control()
   
    i=i+1    
    #%%
    GILoSA.Passive()


# %%
