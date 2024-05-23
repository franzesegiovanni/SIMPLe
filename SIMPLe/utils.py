import numpy as np
def slerp_sat(q1, q2, theta_max_perc): 
    '''
    This function goes to q2 from q1 but with set maximum theta
    '''
    theta_max=theta_max_perc*np.pi/2
    # if np.shape(q1)!=(1,4) or np.shape(q2)!=(1,4):
    #     print("Wrong dimensions of q1 or q2")
    q1=q1.reshape(4)
    q2=q2.reshape(4)
    q1=q1/np.sqrt(np.sum(q1**2))
    q2=q2/np.sqrt(np.sum(q2**2))
    inner=np.inner(q1,q2)
    if inner<0:
        q2=-q2
    theta= np.arccos(np.abs(inner)) 
    q_slerp=np.copy(q2)
    if theta>theta_max:
        q_slerp[0]=(np.sin(theta-theta_max)*q1[0]+np.sin(theta_max)*q2[0])/np.sin(theta)
        q_slerp[1]=(np.sin(theta-theta_max)*q1[1]+np.sin(theta_max)*q2[1])/np.sin(theta)
        q_slerp[2]=(np.sin(theta-theta_max)*q1[2]+np.sin(theta_max)*q2[2])/np.sin(theta)
        q_slerp[3]=(np.sin(theta-theta_max)*q1[3]+np.sin(theta_max)*q2[3])/np.sin(theta)
    return q_slerp 