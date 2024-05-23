import numpy as np
from .utils import slerp_sat
import rospy
from .panda import Panda
import pathlib
from geometry_msgs.msg import PoseStamped
class SIMPLe(Panda):

    def __init__(self):
        super(SIMPLe, self).__init__()
        # stiffness parameters
        self.K_lin = 600
        self.dK_min = 0.0
        self.null_stiff=[0.0]
        # maximum attractor distance along each axis
        # uncertainty threshold at which stiffness is automatically reduced
        # maximum force of the gradient

        self.rec_freq = 20  # [Hz]
        self.control_freq=20 # [Hz]
        self.r_control=rospy.Rate(self.control_freq)
        self.r_rec=rospy.Rate(self.rec_freq)
        self.speed=1 # if this variable is how much faster the motion is going to be executed with respect to the recorded one. It can be any potiviie number, does not need to be an integer. It can also be smaller than 1, in this case the motion is going to be slower than the recorded one.
        self.look_ahead=3
    def Record_Demonstration(self):
        self.Kinesthetic_Demonstration()
        print('Recording ended.')
        save_demo = input("Do you want to keep this demonstration? [y/n] \n")
        if save_demo.lower()=='y':
            self.training_traj=np.empty((0,3))
            self.training_ori=np.empty((0,4))
            self.training_gripper=np.empty((0,1))

            self.training_traj=np.vstack([self.training_traj,self.recorded_traj])
            self.training_ori=np.vstack([self.training_ori,self.recorded_ori])
            self.training_gripper=np.vstack([self.training_gripper,self.recorded_gripper])
            print("Demo Saved")
        else:
            print("Demo Discarded")

    def go_to_start(self):
        print("Reset to the starting cartesian position")
        start = PoseStamped()
        self.home_gripper()

        start.pose.position.x = self.training_traj[0,0]
        start.pose.position.y = self.training_traj[0,1]
        start.pose.position.z = self.training_traj[0,2]
        
        start.pose.orientation.w = self.training_ori[0,0] 
        start.pose.orientation.x = self.training_ori[0,1] 
        start.pose.orientation.y = self.training_ori[0,2] 
        start.pose.orientation.z = self.training_ori[0,3] 
        self.go_to_pose(start)        

    def GGP(self):
        n_samples=self.training_traj.shape[0]

        labda_position=0.05 #lengthscale of the position    
        lamda_time=0.05 #lenghtscale of the time
        lambda_index=self.rec_freq*lamda_time #convert the lenghtscale to work with indexes
        
        # we use an exponential kernel. The uncertainty can be estimated as simga= 1- exp(- (position_error)/lambda_error - (time_error)/lambda_time)
        #We consider uncertainty points that have a distance of at least 2 times the sum of the normalized errors with the lengthscales
        sigma_treshold= 1 - np.exp(-2) 
        #calcolation of correlation in space
        position_error= np.linalg.norm(self.training_traj - np.array(self.cart_pos), axis=1)/labda_position

        #calcolation of correlation in time
        index=np.min([self.mu_index+self.look_ahead, n_samples-1])

        index_error= np.abs(np.arange(n_samples)-index)/lambda_index
        index_error_clip= np.clip(index_error, 0, 1) # we saturate the time error, to avoid that points that are far away in time cannot be activated in case of perturbation of the robot

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
            self.mu_index = self.mu_index+ self.speed*np.sign((int(np.argmax(k_start_time_position))- self.mu_index))

        control_index= np.floor(self.mu_index+self.look_ahead).astype(int) # the floor is used to sed the integer part of the number since we are looking for indexes.
        control_index=np.min([control_index, n_samples-1])

        return  control_index, beta

    def save(self, file='last'):
        np.savez(str(pathlib.Path().resolve())+'/data/'+str(file)+'.npz', 
        training_traj=self.training_traj,
        training_ori = self.training_ori,
        training_gripper=self.training_gripper)
        print('Training data saved shape')
        print('Traning ori')
        print(np.shape(self.training_ori))
        print('Training traj')
        print(np.shape(self.training_traj))  
        print('Training gripper')
        print(np.shape(self.training_gripper))

    def load(self, file='last'):
        data =np.load(str(pathlib.Path().resolve())+'/data/'+str(file)+'.npz')

        self.training_traj=data['training_traj']
        self.training_ori=data['training_ori']
        self.training_gripper=data['training_gripper'] 


    def initialize_mu_index(self):
        position_error= np.linalg.norm(self.training_traj - np.array(self.cart_pos), axis=1)
        self.mu_index = int(np.argmin(position_error))

    def Interactive_Control(self):
        print("Press Esc to stop.")
        self.end=False
        while not self.end:            

            self.step()    

            self.r_control.sleep()

    def control(self):
        self.initialize_mu_index()
        self.Interactive_Control()

    def step(self):
        
        i, beta = self.GGP()

        pos_goal  = self.training_traj[i,:]
        pos_goal=self.cart_pos+ np.clip([pos_goal[0]-self.cart_pos[0],pos_goal[1]-self.cart_pos[1],pos_goal[2]-self.cart_pos[2]],-0.05,0.05)
        quat_goal = self.training_ori[i,:]
        quat_goal=slerp_sat(self.cart_ori, quat_goal, 0.1)
        gripper_goal=self.training_gripper[i,0]
        
        self.set_attractor(pos_goal,quat_goal)
        self.move_gripper(gripper_goal) #TODO write a better logic for the gripper 
            
        K_lin_scaled =beta*self.K_lin
        K_ori_scaled =beta*self.K_ori
        pos_stiff = [K_lin_scaled,K_lin_scaled,K_lin_scaled]
        rot_stiff = [K_ori_scaled,K_ori_scaled,K_ori_scaled]

        self.set_stiffness(pos_stiff, rot_stiff, self.null_stiff)    
