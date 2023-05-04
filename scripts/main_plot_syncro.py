# %%
import matplotlib.pyplot as plt
import numpy as np
import pathlib

# from recording_node import Recorder
def load(filename):
    data = np.load(str(pathlib.Path().resolve()) + '/recording_syncro/' + str(filename) + '.npz')
    return data

# %%
# filename = "19_16_33_45"
# data = load(filename=filename)
# time_1 = np.linspace(0, data['recorded_cart_pose'].shape[1], data['recorded_cart_pose'].shape[1])/100

# 'cart_pose: x, y, z, qz, qy, qz, qw'
# right_cart_pose_1 = data["recorded_cart_pose"][:7]
# left_cart_pose_1 = data['recorded_cart_pose'][7:]

# right_attractor_pose_1 = data['recorded_attractor_pose'][:7]
# left_attractor_pose_1 = data['recorded_attractor_pose'][7:]

# right_attractor_stiffness_1 = data['recorded_stiffness'][:7]
# left_attractor_stiffness_1 = data['recorded_stiffness'][7:]
# filename = "19_16_34_33"
# data = load(filename=filename)
# time_2 = np.linspace(0, data['recorded_cart_pose'].shape[1], data['recorded_cart_pose'].shape[1])/100

# 'cart_pose: x, y, z, qz, qy, qz, qw'
# right_cart_pose_2 = data["recorded_cart_pose"][:7]
# left_cart_pose_2 = data['recorded_cart_pose'][7:]

# right_attractor_pose_2 = data['recorded_attractor_pose'][:7]
# left_attractor_pose_2 = data['recorded_attractor_pose'][7:]

# right_attractor_stiffness_2 = data['recorded_stiffness'][:7]
# left_attractor_stiffness_2 = data['recorded_stiffness'][7:]
# plt.figure(1)
# plt.ylabel('x')
# plt.xlabel('time')
# plt.plot(time_1, left_cart_pose_1[0], label='data1', linewidth=2)
# plt.plot(time_2, right_cart_pose_2[0], label='data1', linewidth=2)
# plt.legend(['Left', 'Right'])
# plt.title('Independent Demonstrations')
# plt.grid()
# plt.savefig("/home/oem/Desktop/interactive-bayesian-robot-learning/figure/recorded_demo_x.png" , dpi=300)
# # plt.show()

# plt.figure(2)
# plt.ylabel('y')
# plt.xlabel('time')
# plt.plot(time_1, left_cart_pose_1[1], label='data1', linewidth=2)
# plt.plot(time_2, right_cart_pose_2[1], label='data1', linewidth=2)
# plt.legend(['Left', 'Right'])
# plt.title('Independent Demonstrations')
# plt.grid()
# plt.savefig("/home/oem/Desktop/interactive-bayesian-robot-learning/figure/recorded_demo_y.png" , dpi=300)
# # plt.show()

# plt.figure(3)
# plt.ylabel('z')
# plt.xlabel('time')
# plt.plot(time_1, left_cart_pose_1[2], label='data1', linewidth=2)
# plt.plot(time_2, right_cart_pose_2[2], label='data1', linewidth=2)
# plt.legend(['Left', 'Right'])
# plt.title('Independent Demonstrations')
# plt.grid()
# plt.savefig("/home/oem/Desktop/interactive-bayesian-robot-learning/figure/recorded_demo_z.png" , dpi=300)
# #%% Now we plot the syncronization phase 
# # %%
# filename = "19_16_39_22"
# data = load(filename=filename)
# time_1 = np.linspace(0, data['recorded_cart_pose'].shape[1], data['recorded_cart_pose'].shape[1])/100

# 'cart_pose: x, y, z, qz, qy, qz, qw'
# right_cart_pose_1 = data["recorded_cart_pose"][:7]
# left_cart_pose_1 = data['recorded_cart_pose'][7:]

# right_attractor_pose_1 = data['recorded_attractor_pose'][:7]
# left_attractor_pose_1 = data['recorded_attractor_pose'][7:]

# right_attractor_stiffness_1 = data['recorded_stiffness'][:7]
# left_attractor_stiffness_1 = data['recorded_stiffness'][7:]
# plt.figure(4)
# plt.ylabel('x')
# plt.xlabel('time')
# plt.plot(time_1, left_cart_pose_1[0], label='data1', linewidth=2)
# plt.plot(time_1, right_cart_pose_1[0], label='data1', linewidth=2)
# plt.legend(['Left', 'Right'])
# plt.title('Syncronization')
# plt.grid()
# plt.savefig("/home/oem/Desktop/interactive-bayesian-robot-learning/figure/syncro_demo_x.png" , dpi=300)
# # plt.show()

# plt.figure(5)
# plt.ylabel('y')
# plt.xlabel('time')
# plt.plot(time_1, left_cart_pose_1[1], label='data1', linewidth=2)
# plt.plot(time_1, right_cart_pose_1[1], label='data1', linewidth=2)
# plt.legend(['Left', 'Right'])
# plt.title('Syncronization')
# plt.grid()
# plt.savefig("/home/oem/Desktop/interactive-bayesian-robot-learning/figure/syncro_demo_y.png" , dpi=300)
# # plt.show()

# plt.figure(6)
# plt.ylabel('z')
# plt.xlabel('time')
# plt.plot(time_1, left_cart_pose_1[2], label='data1', linewidth=2)
# plt.plot(time_1, right_cart_pose_1[2], label='data1', linewidth=2)
# plt.legend(['Left', 'Right'])
# plt.title('Syncronization')
# plt.grid()
# plt.savefig("/home/oem/Desktop/interactive-bayesian-robot-learning/figure/syncro_demo_z.png" , dpi=300)

# # %% Flot external forces ['recorded_force_torque']
# filename = "19_16_39_22"
# data = load(filename=filename)
# time_1 = np.linspace(0, data['recorded_cart_pose'].shape[1], data['recorded_cart_pose'].shape[1])/100

# 'cart_pose: x, y, z, qz, qy, qz, qw'
# right_cart_force_1 = data['recorded_force_torque'][:3]
# left_cart_force_1 = data['recorded_force_torque'][6:9]

# right_attractor_stiffness_1 = data['recorded_stiffness'][:7]
# left_attractor_stiffness_1 = data['recorded_stiffness'][7:]
# plt.figure()
# plt.ylabel('x')
# plt.xlabel('time')
# plt.plot(time_1, left_cart_force_1[0], label='data1', linewidth=2)
# plt.plot(time_1, right_cart_force_1[0], label='data1', linewidth=2)
# plt.legend(['Left', 'Right'])
# plt.title('Force')
# plt.grid()
# plt.savefig("/home/oem/Desktop/interactive-bayesian-robot-learning/figure/force_sycro_x.png" , dpi=300)
# # plt.show()

# plt.figure()
# plt.ylabel('y')
# plt.xlabel('time')
# plt.plot(time_1, left_cart_force_1[1], label='data1', linewidth=2)
# plt.plot(time_1, right_cart_force_1[1], label='data1', linewidth=2)
# plt.legend(['Left', 'Right'])
# plt.title('Force')
# plt.grid()
# plt.savefig("/home/oem/Desktop/interactive-bayesian-robot-learning/figure/force_sycro_y.png" , dpi=300)
# # plt.show()

# plt.figure()
# plt.ylabel('z')
# plt.xlabel('time')
# plt.plot(time_1, left_cart_force_1[2], label='data1', linewidth=2)
# plt.plot(time_1, right_cart_force_1[2], label='data1', linewidth=2)
# plt.legend(['Left', 'Right'])
# plt.title('Force')
# plt.grid()
# plt.savefig("/home/oem/Desktop/interactive-bayesian-robot-learning/figure/force_sycro_z.png" , dpi=300)

# norm_force_left=np.sqrt(left_cart_force_1[0]**2+left_cart_force_1[1]**2+left_cart_force_1[2]**2)
# norm_force_right=np.sqrt(right_cart_force_1[0]**2+right_cart_force_1[1]**2+right_cart_force_1[2]**2)
# plt.figure()
# plt.ylabel('ForceNorm')
# plt.xlabel('time')
# plt.plot(time_1, norm_force_left[:], label='data1', linewidth=2)
# plt.plot(time_1, norm_force_right[:], label='data1', linewidth=2)
# plt.legend(['Left', 'Right'])
# plt.title('Force Norm')
# plt.grid()
# plt.savefig("/home/oem/Desktop/interactive-bayesian-robot-learning/figure/force_sycro_norm.png" , dpi=300)
# plt.show()

# #%% PRINT SUBPLOT
fig, ax = plt.subplots(1, 3, sharex='col', sharey='row')

filename = "19_16_33_45"
data = load(filename=filename)
time_1 = np.linspace(0, data['recorded_cart_pose'].shape[1], data['recorded_cart_pose'].shape[1])/100

'cart_pose: x, y, z, qz, qy, qz, qw'
right_cart_pose_1 = data["recorded_cart_pose"][:7]
left_cart_pose_1 = data['recorded_cart_pose'][7:]

right_attractor_pose_1 = data['recorded_attractor_pose'][:7]
left_attractor_pose_1 = data['recorded_attractor_pose'][7:]

right_attractor_stiffness_1 = data['recorded_stiffness'][:7]
left_attractor_stiffness_1 = data['recorded_stiffness'][7:]
filename = "19_16_34_33"
data = load(filename=filename)
time_2 = np.linspace(0, data['recorded_cart_pose'].shape[1], data['recorded_cart_pose'].shape[1])/100

'cart_pose: x, y, z, qz, qy, qz, qw'
right_cart_pose_2 = data["recorded_cart_pose"][:7]
left_cart_pose_2 = data['recorded_cart_pose'][7:]

right_attractor_pose_2 = data['recorded_attractor_pose'][:7]
left_attractor_pose_2 = data['recorded_attractor_pose'][7:]

right_attractor_stiffness_2 = data['recorded_stiffness'][:7]
left_attractor_stiffness_2 = data['recorded_stiffness'][7:]
# plt.figure(1)
# ax[0].set_ylabel('x')
ax[0].set_xlabel('time[s]')
ax[0].plot(time_1, left_cart_pose_1[0], '--', label='data1', linewidth=2)
ax[0].plot(time_2, right_cart_pose_2[0], '--', label='data1', linewidth=2)

# ax[0].title('Independent Demonstrations')

# plt.savefig("/home/oem/Desktop/interactive-bayesian-robot-learning/figure/recorded_demo_x.png" , dpi=300)
# plt.show()

# plt.figure(2)
# ax[1].set_ylabel('y')
ax[1].set_xlabel('time[s]')
ax[1].plot(time_1, left_cart_pose_1[1], '--', label='data1', linewidth=2)
ax[1].plot(time_2, right_cart_pose_2[1], '--', label='data1', linewidth=2)
# ax[0].title('Independent Demonstrations')

# plt.savefig("/home/oem/Desktop/interactive-bayesian-robot-learning/figure/recorded_demo_y.png" , dpi=300)
# plt.show()

# plt.figure(3)
# plt.ylabel('z')
plt.xlabel('time[s]')
ax[2].plot(time_1, left_cart_pose_1[2], '--', label='data1', linewidth=2)
ax[2].plot(time_2, right_cart_pose_2[2], '--', label='data1', linewidth=2)
# plt.title('Independent Demonstrations')

# plt.savefig("/home/oem/Desktop/interactive-bayesian-robot-learning/figure/recorded_demo_z.png" , dpi=300)
#%% Now we plot the syncronization phase 
# %%
filename = "19_16_39_22"
data = load(filename=filename)
time_3 = np.linspace(0, data['recorded_cart_pose'].shape[1], data['recorded_cart_pose'].shape[1])/100

'cart_pose: x, y, z, qz, qy, qz, qw'
right_cart_pose_3 = data["recorded_cart_pose"][:7]
left_cart_pose_3 = data['recorded_cart_pose'][7:]

right_attractor_pose_3 = data['recorded_attractor_pose'][:7]
left_attractor_pose_3 = data['recorded_attractor_pose'][7:]

right_attractor_stiffness_3 = data['recorded_stiffness'][:7]
left_attractor_stiffness_3 = data['recorded_stiffness'][7:]

ax[0].plot(time_3, left_cart_pose_3[0], label='data1', linewidth=2)
ax[0].plot(time_3, right_cart_pose_3[0], label='data1', linewidth=2)
ax[0].legend(['Left_demo', 'Right_demo', 'Left_sync', 'Right_sync'] )
# plt.title('Syncronization')

ax[0].grid()
ax[0].set_ylabel('x')
# plt.savefig("/home/oem/Desktop/interactive-bayesian-robot-learning/figure/syncro_demo_x.png" , dpi=300)
# plt.show()

ax[1].plot(time_3, left_cart_pose_3[1], label='data1', linewidth=2)
ax[1].plot(time_3, right_cart_pose_3[1], label='data1', linewidth=2)
ax[1].legend(['Left_demo', 'Right_demo', 'Left_sync', 'Right_sync'] )
ax[1].set_ylabel('y')
# plt.title('Syncronization')
ax[1].grid()
# plt.savefig("/home/oem/Desktop/interactive-bayesian-robot-learning/figure/syncro_demo_y.png" , dpi=300)
# plt.show()

ax[2].plot(time_3, left_cart_pose_3[2], label='data1', linewidth=2)
ax[2].plot(time_3, right_cart_pose_3[2], label='data1', linewidth=2)
ax[2].legend(['Left_demo', 'Right_demo', 'Left_sync', 'Right_sync'] )
ax[2].set_ylabel('z')
# plt.title('Syncronization')
ax[2].grid()
# plt.savefig("/home/oem/Desktop/interactive-bayesian-robot-learning/figure/syncro_demo_z.png" , dpi=300)
plt.show()