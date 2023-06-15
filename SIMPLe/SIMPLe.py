from ILoSA import ILoSA #you need to pip install ILoSA first 

class SIMPLe(ILoSA):

    def __init__(self):
        super(ILoSA, self).__init__()
        self.rec_freq = 30  # [Hz]
        self.control_freq=30 # [Hz]

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
        
        i, beta = GGP()

        pos_goal  = self.training_traj[i,:]
        quat_goal = self.training_ori[i,:]
        quat_goal=slerp_sat(self.cart_ori, quat_goal, 0.1)
        gripper_goal=self.training_gripper[i,0]
        
        self.set_attractor(pos_goal,quat_goal)
        self.move_gripper(gripper_goal)
            
        pos_stiff = self.K_mean*beta*np.ones(1,3) 
        rot_stiff = self.K_ori*beta*np.ones(1,3)  

        self.set_stiffness(pos_stiff, rot_stiff, null_stiff)    