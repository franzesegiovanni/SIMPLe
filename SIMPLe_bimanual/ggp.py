import numpy as np

class GGP():
    def __init__(self, execution_traj, control_frequancy=30,  labda_position=0.05, lamda_time=0.05, look_ahead=3):
        self.look_ahead=look_ahead # how many steps forward is the attractor for any element of the graph

        self.labda_position=labda_position
        self.lambda_index=control_frequancy*lamda_time 

        self.mu_index=0

        self.execution_traj=execution_traj
        
        self.n_samples= self.execution_traj.shape[1]
        
    def step(self, cart_pos):

        
        sigma_treshold= 1 - np.exp(-2)
        #calcolation of correlation in space
        position_error= np.linalg.norm(self.execution_traj.T - np.array(cart_pos), axis=1)/self.labda_position

        #calcolation of correlation in time
        index=np.min([self.mu_index+self.look_ahead, self.n_samples-1])
        index_error= np.abs(np.arange(self.n_samples)-index)/self.lambda_index
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

        i = np.min([self.mu_index+int(self.look_ahead), self.n_samples-1]) 

        return  i, beta
    

