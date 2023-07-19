from ILoSA import ILoSA #you need to pip install ILoSA first 
import numpy as np
from ILoSA import InteractiveGP
from ILoSA.data_prep import slerp_sat
import pickle
from sklearn.gaussian_process.kernels import RBF, Matern, WhiteKernel, ConstantKernel as C
import rospy


class SIMPLe(ILoSA):

    def __init__(self):
        super(SIMPLe, self).__init__()
        self.rec_freq = 20  # [Hz]
        self.control_freq=20 # [Hz]
        self.r_control=rospy.Rate(self.control_freq)
        self.r_rec=rospy.Rate(self.rec_freq)
    def Train_GPs(self):
        print("SIMPLe does not need to be trained")
        if len(self.nullspace_traj)>0 and len(self.nullspace_joints)>0:
            print("Training of Nullspace")
            kernel = C(constant_value = 0.1, constant_value_bounds=[0.0005, self.attractor_lim]) * RBF(length_scale=[0.1, 0.1, 0.1], length_scale_bounds=[0.025, 0.1]) + WhiteKernel(0.00025, [0.0001, 0.0005]) 
            self.NullSpaceControl=InteractiveGP(X=self.nullspace_traj, Y=self.nullspace_joints, y_lim=[-self.attractor_lim, self.attractor_lim], kernel=kernel, n_restarts_optimizer=20)
            self.NullSpaceControl.fit()
            with open('models/nullspace.pkl','wb') as nullspace:
                pickle.dump(self.NullSpaceControl,nullspace)
        else: 
            print('No Null Space Control Policy Learned')    



    def GGP(self):
        n_samples=self.training_traj.shape[0]
        look_ahead=3 # how many steps forward is the attractor for any element of the graph

        labda_position=0.05
        lamda_time=0.05
        lambda_index=self.rec_freq*lamda_time
        
        sigma_treshold= 1 - np.exp(-2)
        #calcolation of correlation in space
        position_error= np.linalg.norm(self.training_traj - np.array(self.cart_pos), axis=1)/labda_position

        #calcolation of correlation in time
        index_error= np.abs(np.arange(n_samples)-self.index)/lambda_index
        index_error_clip= np.clip(index_error, 0, 1)

        # Calculate the product of the two correlation vectors
        k_start_time_position=np.exp(-position_error-index_error_clip)

        # Compute the uncertainty only as a function of the correlation in space and time
        sigma_position_time= 1- np.max(k_start_time_position)

        # Compute the scaling factor for the stiffness Eq 15
        if sigma_position_time > sigma_treshold: 
            beta= (1-sigma_position_time)/(1-sigma_treshold)
            self.mu_index = int(np.argmax(k_start_time_position))
        else:
            beta=1
            self.mu_index = int(self.mu_index+ 1.0*np.sign((int(np.argmax(k_start_time_position))- self.mu_index)))

        if any(np.abs(np.array(self.feedback)) > 0.05): # this avoids to activate the feedback on noise joystick
            print("Received Feedback")
            self.training_traj=self.training_traj+ np.array(self.feedback)*k_start_time_position
             
        self.index=np.min([self.mu_index+look_ahead, n_samples-1])

        return  self.index, beta

    def step(self):
        
        i, beta = self.GGP()

        pos_goal  = self.training_traj[i,:]
        pos_goal=self.cart_pos+ np.clip([pos_goal[0]-self.cart_pos[0],pos_goal[1]-self.cart_pos[1],pos_goal[2]-self.cart_pos[2]],-0.05,0.05)
        quat_goal = self.training_ori[i,:]
        quat_goal=slerp_sat(self.cart_ori, quat_goal, 0.1)
        gripper_goal=self.training_gripper[i,0]
        
        self.set_attractor(pos_goal,quat_goal)
        self.grasp_gripper(gripper_goal)
            
        K_lin_scaled =beta*self.K_mean
        K_ori_scaled =beta*self.K_ori
        pos_stiff = [K_lin_scaled,K_lin_scaled,K_lin_scaled]
        rot_stiff = [K_ori_scaled,K_ori_scaled,K_ori_scaled]
        # self.pos_stiff = self.K_mean*beta*np.ones([1,3]) 
        # self.rot_stiff = self.K_ori*beta*np.ones([1,3])  

        self.set_stiffness(pos_stiff, rot_stiff, self.null_stiff)    


    def convert_distribution_to_position_array(self):
        target_array=np.array([], dtype=np.int64).reshape(0,3)
        for detection_target_in_camera in self.target_distribution:
            detection_target = self.transform_in_base(detection_target_in_camera.pose.pose.pose)
            t=np.array([detection_target.pose.position.x,detection_target.pose.position.y,detection_target.pose.position.z])
            target_array=np.vstack((target_array,t))
        return target_array


    def convert_distribution_to_orientation_array(self):
        target_array=np.array([], dtype=np.int64).reshape(0,4)
        for detection_target_in_camera in self.target_distribution:
            detection_target = self.transform_in_base(detection_target_in_camera.pose.pose.pose)
            t=np.array([detection_target.pose.orientation.w,detection_target.pose.orientation.x,detection_target.pose.orientation.y,detection_target.pose.orientation.z])
            target_array=np.vstack((target_array,t))
        return target_array
