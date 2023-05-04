# %%
import matplotlib.pyplot as plt
import numpy as np
import pathlib

# from recording_node import Recorder
def load(filename):
    data = np.load(str(pathlib.Path().resolve()) + '/pick_place_official/' + str(filename) + '.npz')
    return data


# %%
filename = "23_14_09_09"
data = load(filename=filename)
time_demo_right_1 = np.linspace(0, data['recorded_cart_pose'].shape[1], data['recorded_cart_pose'].shape[1])/100

'cart_pose: x, y, z, qz, qy, qz, qw'
right_cart_pose_demo_1 = data["recorded_cart_pose"][:7]

right_attractor_pose__demo_1 = data['recorded_attractor_pose'][:7]

# right_attractor_stiffness_1 = data['recorded_stiffness'][:7]
# plt.figure(1)
# # plt.ylabel('x')
# plt.xlabel('time[s]')
# plt.plot(time_1, right_cart_pose_1[0], label='data1', linewidth=2)
# plt.plot(time_1, right_cart_pose_1[1], label='data1', linewidth=2)
# plt.plot(time_1, right_cart_pose_1[2], label='data1', linewidth=2)
# plt.legend(['x', 'y', 'z'])
# plt.title('Right Demonstration')
# plt.grid()
# plt.savefig("/home/oem/Desktop/interactive-bayesian-robot-learning/figure/recorded_demo_x.png" , dpi=300)
# plt.show()

filename = "23_14_10_09"
data = load(filename=filename)
time_demo_left_1 = np.linspace(0, data['recorded_cart_pose'].shape[1], data['recorded_cart_pose'].shape[1])/100

'cart_pose: x, y, z, qz, qy, qz, qw'
left_cart_pose_demo_1 = data["recorded_cart_pose"][7:]

left_attractor_pose_demo_1 = data['recorded_attractor_pose'][7:]

# right_attractor_stiffness_1 = data['recorded_stiffness'][:7]
# plt.figure()
# # plt.ylabel('x')
# plt.xlabel('time[s]')
# plt.plot(time_1, left_cart_pose_1[0], label='data1', linewidth=2)
# plt.plot(time_1, left_cart_pose_1[1], label='data1', linewidth=2)
# plt.plot(time_1, left_cart_pose_1[2], label='data1', linewidth=2)
# plt.legend(['x', 'y', 'z'])
# plt.title('Left Demonstration')
# plt.grid()
# plt.savefig("/home/oem/Desktop/interactive-bayesian-robot-learning/figure/recorded_demo_x.png" , dpi=300)
# plt.show()

filename = "23_14_11_09"
data = load(filename=filename)
time_1 = np.linspace(0, data['recorded_cart_pose'].shape[1], data['recorded_cart_pose'].shape[1])/100

'cart_pose: x, y, z, qz, qy, qz, qw'
left_cart_pose_1 = data["recorded_cart_pose"][7:]

left_attractor_pose_1 = data['recorded_attractor_pose'][7:]

left_attractor_stiffness_1 = data['recorded_stiffness'][7:]

right_cart_pose_1 = data["recorded_cart_pose"][:7]

right_attractor_pose_1 = data['recorded_attractor_pose'][:7]

right_attractor_stiffness_1 = data['recorded_stiffness'][:7]

right_cart_force_1 = data['recorded_force_torque'][:3]
left_cart_force_1 = data['recorded_force_torque'][6:9]
norm_force_left=np.sqrt(left_cart_force_1[0]**2+left_cart_force_1[1]**2+left_cart_force_1[2]**2)
norm_force_right=np.sqrt(right_cart_force_1[0]**2+right_cart_force_1[1]**2+right_cart_force_1[2]**2)

fig, ax = plt.subplots(3, 1, sharex='col', sharey='row')
# plt.figure()
# plt.ylabel('x')
# ax[0].xlabel('time')
ax[0].plot(time_1, left_cart_pose_1[0], label='data1', linewidth=2)
ax[0].plot(time_1, left_cart_pose_1[1], label='data1', linewidth=2)
ax[0].plot(time_1, left_cart_pose_1[2], label='data1', linewidth=2)

ax[0].plot(time_demo_left_1, left_cart_pose_demo_1[0],  '--', label='data1', linewidth=2)
ax[0].plot(time_demo_left_1, left_cart_pose_demo_1[1], '--',  label='data1', linewidth=2)
ax[0].plot(time_demo_left_1, left_cart_pose_demo_1[2], '--',  label='data1', linewidth=2)

ax[0].legend(['x', 'y', 'z', 'demo_x', 'demo_y', 'demo_z'])
ax[0].set_title('Left Position')
ax[0].grid()
# plt.savefig("/home/oem/Desktop/interactive-bayesian-robot-learning/figure/recorded_demo_x.png" , dpi=300)
# plt.figure()
# plt.ylabel('x')
# ax[1].xlabel('time')
ax[1].plot(time_1, norm_force_left, label='data1', linewidth=2)
ax[1].legend(['N'])
ax[1].set_title('Left External force')
ax[1].grid()

ax[2].plot(time_1[1:], left_attractor_stiffness_1[0][1:], label='data1', linewidth=2)
# ax[1].plot(time_1, left_attractor_stiffness_1[1], label='data1', linewidth=2)
# ax[1].plot(time_1, left_attractor_stiffness_1[2], label='data1', linewidth=2)
ax[2].legend(['N/m'])
ax[2].set_title('Left Cartesian Stiffeness')
ax[2].grid()
ax[2].set_xlabel('time [s]')
# plt.savefig("/home/oem/Desktop/interactive-bayesian-robot-learning/figure/recorded_demo_x.png" , dpi=300)
# plt.figure()
# plt.ylabel('x')
# ax[1].xlabel('time')

fig, ax = plt.subplots(3, 1, sharex='col', sharey='row')
# plt.figure()
# plt.ylabel('x')
# ax[0].xlabel('time')
ax[0].plot(time_1, right_cart_pose_1[0], label='data1', linewidth=2)
ax[0].plot(time_1, right_cart_pose_1[1], label='data1', linewidth=2)
ax[0].plot(time_1, right_cart_pose_1[2], label='data1', linewidth=2)
ax[0].plot(time_demo_right_1, right_cart_pose_demo_1[0],  '--', label='data1', linewidth=2)
ax[0].plot(time_demo_right_1, right_cart_pose_demo_1[1], '--',  label='data1', linewidth=2)
ax[0].plot(time_demo_right_1, right_cart_pose_demo_1[2], '--',  label='data1', linewidth=2)
ax[0].legend(['x', 'y', 'z', 'demo_x', 'demo_y', 'demo_z'])
ax[0].set_title('Right Position')
ax[0].grid()
# plt.savefig("/home/oem/Desktop/interactive-bayesian-robot-learning/figure/recorded_demo_x.png" , dpi=300)
# plt.figure()
# plt.ylabel('x')
# ax[1].xlabel('time')
ax[1].plot(time_1, norm_force_right, label='data1', linewidth=2)
ax[1].legend(['N'])
ax[1].set_title('Right External force')
ax[1].grid()

ax[2].plot(time_1[1:], right_attractor_stiffness_1[0][1:], label='data1', linewidth=2)
# ax[1].plot(time_1, left_attractor_stiffness_1[1], label='data1', linewidth=2)
# ax[1].plot(time_1, left_attractor_stiffness_1[2], label='data1', linewidth=2)
ax[2].legend(['N/m'])
ax[2].set_title('Right Cartesian Stiffness')
ax[2].grid()
ax[2].set_xlabel('time [s]')
plt.show()
# plt.savefig("/home/oem/Desktop/interactive-bayesian-robot-learning/figure/recorded_demo_x.png" , dpi=300)
# plt.figure()
# plt.ylabel('x')
# ax[1].xlabel('time')
# plt.figure()
# plt.xlabel('time')
# plt.plot(time_1, right_cart_pose_1[0], label='data1', linewidth=2)
# plt.plot(time_1, right_cart_pose_1[1], label='data1', linewidth=2)
# plt.plot(time_1, right_cart_pose_1[2], label='data1', linewidth=2)
# plt.legend(['x', 'y', 'z'])
# plt.title('Right')
# plt.grid()
# # plt.savefig("/home/oem/Desktop/interactive-bayesian-robot-learning/figure/recorded_demo_x.png" , dpi=300)
# plt.figure()
# # plt.ylabel('x')
# plt.xlabel('time')
# plt.plot(time_1, right_attractor_stiffness_1[0], label='data1', linewidth=2)
# # plt.plot(time_1, right_attractor_stiffness_1[1], label='data1', linewidth=2)
# # plt.plot(time_1, right_attractor_stiffness_1[2], label='data1', linewidth=2)
# # plt.legend(['x', 'y', 'z'])
# plt.title('Right')
# plt.grid()

# plt.figure()
# # plt.ylabel('x')
# plt.xlabel('time')
# plt.plot(time_1, norm_force_right, label='data1', linewidth=2)
# # plt.plot(time_1, right_cart_force_1[1], label='data1', linewidth=2)
# # plt.plot(time_1, right_cart_force_1[2], label='data1', linewidth=2)
# # plt.legend(['x', 'y', 'z'])
# plt.title('Right')
# plt.grid()
# # plt.set_size_inches(8.5, 30.5)
# plt.show()
# plt.xlabel('time')
# plt.plot(time_1, right_cart_pose_1[0], label='data1', linewidth=2)
# plt.plot(time_1, right_cart_pose_1[1], label='data1', linewidth=2)
# plt.plot(time_1, right_cart_pose_1[2], label='data1', linewidth=2)
# plt.legend(['x', 'y', 'z'])
# # plt.title('Independent Demonstrations')
# plt.grid()
# # plt.savefig("/home/oem/Desktop/interactive-bayesian-robot-learning/figure/recorded_demo_x.png" , dpi=300)
# plt.figure()
# # plt.ylabel('x')
# plt.xlabel('time')
# plt.plot(time_1, right_attractor_stiffness_1[0], label='data1', linewidth=2)
# plt.plot(time_1, right_attractor_stiffness_1[1], label='data1', linewidth=2)
# plt.plot(time_1, right_attractor_stiffness_1[2], label='data1', linewidth=2)
# plt.legend(['x', 'y', 'z'])
# # plt.title('Independent Demonstrations')
# plt.grid()
# plt.show()

# plt.xlabel('time')
# plt.plot(time_1, right_cart_pose_1[0], label='data1', linewidth=2)
# plt.plot(time_1, right_cart_pose_1[1], label='data1', linewidth=2)
# plt.plot(time_1, right_cart_pose_1[2], label='data1', linewidth=2)
# plt.legend(['x', 'y', 'z'])
# # plt.title('Independent Demonstrations')
# plt.grid()
# # plt.savefig("/home/oem/Desktop/interactive-bayesian-robot-learning/figure/recorded_demo_x.png" , dpi=300)
# plt.figure()
# # plt.ylabel('x')
# plt.xlabel('time')
# plt.plot(time_1, right_attractor_stiffness_1[0], label='data1', linewidth=2)
# plt.plot(time_1, right_attractor_stiffness_1[1], label='data1', linewidth=2)
# plt.plot(time_1, right_attractor_stiffness_1[2], label='data1', linewidth=2)
# plt.legend(['x', 'y', 'z'])
# # plt.title('Independent Demonstrations')
# plt.grid()
# plt.show()

# filename = "23_14_10_09"
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

# plt.figure(1)
# plt.ylabel('x')
# plt.plot(time, left_cart_pose1[0], label='data1', color='r')
# plt.plot(data2_t, left_cart_pose2[0], label='data2')
# plt.legend()

# plt.figure(2)
# plt.ylabel('y')
# plt.plot(time, left_cart_pose1[1], label='data1', color='r')
# plt.plot(data2_t, left_cart_pose2[1], label='data2')
# plt.legend()

# plt.figure(3)
# plt.ylabel('z')
# plt.plot(time, left_cart_pose1[3], label='data1', color='r')
# plt.plot(data2_t, left_cart_pose2[3], label='data2')
# plt.legend()
# # %%
# error_based_force_right = data1['recorded_stiffness'][:3]*(data1['recorded_cart_pose'][:3]-data1['recorded_attractor_pose'][:3])
# error_based_force_left = data1['recorded_stiffness'][7:10]*(data1['recorded_cart_pose'][7:10]-data1['recorded_attractor_pose'][7:10])

# plt.figure(1)
# plt.plot(error_based_force_left[0])

# plt.figure(2)
# plt.plot(data1['recorded_force_torque'][6])
# # %%

# plt.figure(1)
# plt.plot(time, data1["recorded_stiffness"][7])
# plt.plot(data2_t, data2["recorded_stiffness"][7])

# plt.figure(2)
# plt.plot(time, data1["recorded_stiffness"][8])
# plt.plot(data2_t, data2["recorded_stiffness"][8])

# plt.figure(3)
# plt.plot(time, data1["recorded_stiffness"][9])
# plt.plot(data2_t, data2["recorded_stiffness"][9])

# # %%
# fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, sharex=True, figsize=(10,10))

# ax1.plot(time, data1["recorded_stiffness"][8], label='recorded stiffness no correction', color='r')
# ax1.plot(data2_t, data2["recorded_stiffness"][8], label='recorded stiffness with correction', color='b')
# ax2.plot(data2_t, data2["recorded_feedback"][4], label='correction input', color='k')
# ax3.plot(data2_t, data2["recorded_cart_pose"][8]-data2["recorded_attractor_pose"][8], label='distance to attractor', color='k')
# ax4.plot(data2_t, data2['recorded_force_torque'][7], label='externally applied force', color='k')

# ax1.set_ylabel('Stiffness (N/m)')
# ax2.set_ylabel('Magnitude')
# ax3.set_ylabel('Distance (m)')
# ax4.set_ylabel('Force (N)')

# ax1.set_ylim(550,900)
# ax2.set_ylim(-1, 0.6)
# ax1.legend()
# ax2.legend()
# ax3.legend()
# ax4.legend()

# %%
