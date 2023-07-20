"""
Authors: Giovanni Franzese and Ravi Prakash
Email: g.franzese@tudelft.nl
Cognitive Robotics, TU Delft
This code is part of TERI (TEaching Robots Interactively) project
"""
#%%
from modules import GILoSA_tag
import time
from geometry_msgs.msg import PoseStamped
import rospy
from sklearn.gaussian_process.kernels import RBF, Matern,WhiteKernel, ConstantKernel as C
import numpy as np
import pickle
#%%
if __name__ == '__main__':
    rospy.init_node('GILoSA', anonymous=True)
    GILoSA=GILoSA_tag()
    GILoSA.connect_ROS()

    time.sleep(1)

  
    # Dressing view
    GILoSA.view_marker = PoseStamped()
    GILoSA.view_marker.header.frame_id = "panda_link0"
    GILoSA.view_marker.pose.position.x = 0.08143687722855915
    GILoSA.view_marker.pose.position.y = 0.31402779786395074
    GILoSA.view_marker.pose.position.z = 0.8247450387759941
    GILoSA.view_marker.pose.orientation.w =0.1649396209403439
    GILoSA.view_marker.pose.orientation.x =  0.876597038344831
    GILoSA.view_marker.pose.orientation.y =  -0.22170860567236517
    GILoSA.view_marker.pose.orientation.z =  0.39397046435209987







    #%%
    time.sleep(1)
    print("Record of the source disributions")
    GILoSA.go_to_pose(GILoSA.view_marker)
    time.sleep(2)

    #%%
    GILoSA.record_source_distribution() 
    print("Source len",len(GILoSA.source_distribution))
    GILoSA.record_target_distribution() 

    print("Save the  source distribution data") 
    GILoSA.save_distributions()   

    f = open("results/dressing/source.pkl","wb")
    # write the python object (dict) to pickle file
    pickle.dump(GILoSA.source_distribution,f)
    # close file
    f.close()  


    #%%
    time.sleep(1)
    print("Record of the cartesian trajectory")
    GILoSA.Record_Demonstration()  
    GILoSA.save()
    #%%
    # Save traj configuration
    f = open("results/dressing/traj_demo.pkl","wb")
    # write the python object (dict) to pickle file
    pickle.dump(GILoSA.training_traj,f)
    # close file
    f.close()
   
    f = open("results/dressing/traj_demo_ori.pkl","wb")
    # write the python object (dict) to pickle file
    pickle.dump(GILoSA.training_ori,f)
    # close file
    f.close()

    #%%
    i=0


    #%%
    time.sleep(1)
    print("Record of the target disributions")
    GILoSA.go_to_pose(GILoSA.view_marker)
    time.sleep(2)

    #%%
    GILoSA.load_distributions()     #load source 
    GILoSA.load()
    #%%
    GILoSA.record_target_distribution() 
    print("Target len", len(GILoSA.target_distribution) )
    # Save start configuration

    #%%
    # Save goal configuration
    f = open("results/dressing/target_"+str(i)+".pkl","wb")
    # write the python object (dict) to pickle file
    pickle.dump(GILoSA.target_distribution,f)
    # close file
    f.close()   
    #%%
    if type(GILoSA.target_distribution) != type(GILoSA.source_distribution):
        raise TypeError("Both the distribution must be a numpy array.")
    elif not(isinstance(GILoSA.target_distribution, np.ndarray)) and not(isinstance(GILoSA.source_distribution, np.ndarray)):
        GILoSA.convert_distribution_to_array(use_orientation=False) #this needs to be a function of every sensor class
        # if you want to use the orientation, you can use
        #GILoSA.convert_distribution_to_array(use_orientation=True) #this needs to be a function of every sensor class

    #%%
    time.sleep(1)
    print("Find the transported policy")
    # GILoSA.kernel_transport=C(0.1,[0.1,0.1]) * RBF(length_scale=[0.3], length_scale_bounds=[0.1,1]) + WhiteKernel(0.000001, [0.000001,0.000001])
    GILoSA.kernel_transport=C(0.1) * RBF(length_scale=[0.1],  length_scale_bounds=[0.05,0.15]) + WhiteKernel(0.0000001, [0.0000001,0.0000001])

    GILoSA.fit_transportation()
    GILoSA.apply_transportation()
    #%%
    # Save traj configuration
    f = open("results/dressing/traj_"+str(i)+".pkl","wb")
    # write the python object (dict) to pickle file
    pickle.dump(GILoSA.training_traj,f)
    # close file
    f.close()


    f = open("results/dressing/traj_ori_"+str(i)+".pkl","wb")
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
