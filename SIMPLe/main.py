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
from sklearn.gaussian_process.kernels import RBF, WhiteKernel, ConstantKernel as C
import numpy as np
import pickle
#%%
if __name__ == '__main__':
    rospy.init_node('GILoSA', anonymous=True)
    GILoSA=SIMPLe()
    GILoSA.connect_ROS()
    time.sleep(1)

    # Top down
    # GILoSA.view_marker = PoseStamped()
    # GILoSA.view_marker.header.frame_id = "panda_link0"
    # GILoSA.view_marker.pose.position.x = 0.24936263473711182
    # GILoSA.view_marker.pose.position.y = 0.009868197140904126
    # GILoSA.view_marker.pose.position.z = 0.7599079962909793
    # GILoSA.view_marker.pose.orientation.w = 0.015628111849494458
    # GILoSA.view_marker.pose.orientation.x =  0.9978447117039875
    # GILoSA.view_marker.pose.orientation.y =  -0.031063472757153612
    # GILoSA.view_marker.pose.orientation.z = 0.05560518089312322

    # Top incline
    GILoSA.view_marker = PoseStamped()
    GILoSA.view_marker.header.frame_id = "panda_link0"
    GILoSA.view_marker.pose.position.x = 0.38560391297366936
    GILoSA.view_marker.pose.position.y = -0.14273974959104221
    GILoSA.view_marker.pose.position.z = 0.6632329274240084
    GILoSA.view_marker.pose.orientation.w = -0.029285083111638313
    GILoSA.view_marker.pose.orientation.x =  0.6920939660028173
    GILoSA.view_marker.pose.orientation.y =  0.7177681013981246
    GILoSA.view_marker.pose.orientation.z = 0.0703419525380227

    GILoSA.task_error = []
    GILoSA.task_success = []


    GILoSA.home_gripper()



    #%%
    time.sleep(1)
    print("Record of the source disributions")
    GILoSA.go_to_pose(GILoSA.view_marker)
    time.sleep(2)

    #%%
    GILoSA.record_source_distribution() 

    #%%
    time.sleep(1)
    print("Record of the cartesian trajectory")
    GILoSA.Record_Demonstration()  

    #%%
    GILoSA.record_target_distribution() 
    GILoSA.demo_final_placement_pos = GILoSA.convert_distribution_to_position_array()
    GILoSA.demo_final_placement_ori = GILoSA.convert_distribution_to_orientation_array()



    #%%
    time.sleep(1)
    print("Load the demo and source distribution data") 
    GILoSA.load()  
    GILoSA.load_distributions()       



    #%%
    time.sleep(1)
    print("Record of the target disributions")
    GILoSA.go_to_pose(GILoSA.view_marker)
    time.sleep(2)


    #%%
    GILoSA.record_target_distribution()  

    #%%
    time.sleep(1)
    print("Save the demo and source distribution data") 
    GILoSA.save()
    GILoSA.save_distributions()   

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
    GILoSA.kernel_transport=C(0.1) * RBF(length_scale=[0.15], length_scale_bounds=[0.1,0.2]) + WhiteKernel(0.000001, [0.000001,0.000001])

    GILoSA.fit_transportation()
    GILoSA.apply_transportation()

    #%%
    time.sleep(1)
    print("Reset to the starting cartesian position")
    start = PoseStamped()
    # GILoSA.home_gripper()

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

    #%%
    time.sleep(1)
    print("Measure task accuracy")
    GILoSA.go_to_pose(GILoSA.view_marker)
    time.sleep(2)
    #%%
    

    # GILoSA.final_placement_pos = GILoSA.convert_distribution_to_position_array()
    # GILoSA.final_placement_ori = GILoSA.convert_distribution_to_orientation_array()



    # pos_error = np.linalg.norm(GILoSA.demo_placement_pos - GILoSA.final_placement_pos[0])

    # # Calculate the dot product between the two quaternions
    # dot_product = np.dot(GILoSA.demo_placement_ori, GILoSA.final_placement_ori[0])

    # # Ensure the dot product is within [-1, 1] to avoid NaN when taking the arccos
    # dot_product = max(min(dot_product, 1), -1)

    # # Calculate the angular distance in radians
    # angular_distance = 2 * np.arccos(abs(dot_product))

    # # Print the result

    # print("pos error", pos_error)

    # print("Angular error:", angular_distance)



    #%%

    # GILoSA.task_error.append(pos_error)


    # for i in range(len(GILoSA.task_error)):
    #     if pos_error < 0.02 and angular_distance < 0.15:
    #         GILoSA.task_success.append(1)
    #     else:
    #         GILoSA.task_success.append(0)


    # #%%

    # print("Average Task Success in Pick and Place Task is : ", sum(GILoSA.task_success), "out of", len(GILoSA.task_success))
    # GILoSA.desired_task_pos_error = [0.254023]*len(GILoSA.task_error)
    # rmse = np.sqrt(np.mean((GILoSA.desired_task_error - GILoSA.task_error) ** 2))
    # print("RMSE in Pick and Place Task is : ", rmse)