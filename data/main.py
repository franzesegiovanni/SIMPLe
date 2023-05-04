"""
Authors: Giovanni Franzese 
Email: g.franzese@tudelft.nl
Cognitive Robotics, TU Delft
This code is part of TERI (TEaching Robots Interactively) project
"""
#%%
import matplotlib
from dual_panda import DualPanda
import time

from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
#%%
if __name__ == '__main__':
    BiManualTeaching=DualPanda()
    #ILoSA.connect_ROS()
    time.sleep(1)
    BiManualTeaching.Panda_left.home_gripper()
    BiManualTeaching.Panda_right.home_gripper()
    time.sleep(1)

#%%
BiManualTeaching.Panda_left.Kinesthetic_Demonstration()
    
# %%
BiManualTeaching.Panda_left.go_to_start()
# %%
BiManualTeaching.Panda_left.execute_traj()
# %%
BiManualTeaching.Panda_left.save()    
#%%
if not BiManualTeaching.Panda_left.pause:
    BiManualTeaching.Panda_left.execute_traj()
    BiManualTeaching.Panda_left.Kinesthetic_Demonstration(active=True)
#%%
BiManualTeaching.Panda_right.Kinesthetic_Demonstration()
# %%
BiManualTeaching.Panda_right.go_to_start()
# %%    
BiManualTeaching.Panda_right.execute_traj()
# %%
BiManualTeaching.Panda_right.save()
# %%
BiManualTeaching.Panda_left.go_to_start()
BiManualTeaching.Panda_right.go_to_start()
# %%  
BiManualTeaching.Panda_right.execute_traj()
BiManualTeaching.Panda_left.execute_traj()
BiManualTeaching.Kinesthetic_Demonstration_BiManual(active=True)
# %%
BiManualTeaching.Kinesthetic_Demonstration_BiManual(active=True)
# %%
BiManualTeaching.Panda_left.home()
BiManualTeaching.Panda_right.home()
#%%
BiManualTeaching.save()
# %%
BiManualTeaching.load()
# %%e
BiManualTeaching.Panda_right.go_to_start()
BiManualTeaching.Panda_left.go_to_start()
#%%    
BiManualTeaching.execute_dual()
# %%
BiManualTeaching.correction_execute_dual()
# %%
BiManualTeaching.enable_correction = False
# %%
BiManualTeaching.Kinesthetic_Demonstration_BiManual()
#%%
BiManualTeaching.go_to_start()
#%%
BiManualTeaching.execute_dual()

#%%
# 3D plot
traj_list = [traj1, traj2]
color_list = ['red', 'gray', 'yellow']
fig = plt.figure(0)
ax = plt.axes(projection='3d')
for i,traj in enumerate(traj_list):
    ax.plot3D(traj[0], traj[1], traj[2], color_list[i])
# %%
ax = plt.axes()
for i,traj in enumerate(traj_list):
    for j, data in enumerate(traj):
        fig = plt.figure(j+1)
        plt.plot(data, color_list[i])
# %%
