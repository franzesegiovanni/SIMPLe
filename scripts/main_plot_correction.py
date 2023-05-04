# %%
import matplotlib.pyplot as plt
import numpy as np
import pathlib

# from recording_node import Recorder
def load(filename):
    data = np.load(str(pathlib.Path().resolve()) + '/recording_correction/' + str(filename) + '.npz')
    return data


# %%
filename = "19_17_33_23"
# filename = "19_17_34_42"
# filename = "19_17_36_07"
# filename = "19_17_37_18" 
# filename = "19_17_41_00"
# filename = "19_17_42_19"
# filename = "19_17_43_14"
data = load(filename=filename)
time_1 = np.linspace(0, data['recorded_cart_pose'].shape[1], data['recorded_cart_pose'].shape[1])/100

'cart_pose: x, y, z, qz, qy, qz, qw'
right_cart_pose_1 = data["recorded_cart_pose"][:7]
left_cart_pose_1 = data['recorded_cart_pose'][7:]

right_attractor_pose_1 = data['recorded_attractor_pose'][:7]
left_attractor_pose_1 = data['recorded_attractor_pose'][7:]

right_attractor_stiffness_1 = data['recorded_stiffness'][:7]
left_attractor_stiffness_1 = data['recorded_stiffness'][7:]

filename = "19_17_41_00"
#filename = "19_17_44_23"
data = load(filename=filename)
time_2 = np.linspace(0, data['recorded_cart_pose'].shape[1], data['recorded_cart_pose'].shape[1])/100

'cart_pose: x, y, z, qz, qy, qz, qw'
right_cart_pose_2 = data["recorded_cart_pose"][:7]
left_cart_pose_2 = data['recorded_cart_pose'][7:]

right_attractor_pose_2 = data['recorded_attractor_pose'][:7]
left_attractor_pose_2 = data['recorded_attractor_pose'][7:]

right_attractor_stiffness_2 = data['recorded_stiffness'][:7]
left_attractor_stiffness_2 = data['recorded_stiffness'][7:]

plt.figure()
plt.ylabel('x')
plt.xlabel('time')
plt.plot(time_1, left_cart_pose_1[0], label='data1', linewidth=2, color='black')
plt.plot(time_1, right_cart_pose_1[0], label='data1', linewidth=2, color='red')
plt.plot(time_2, left_cart_pose_2[0], '--', label='data1', linewidth=2, color='black')
plt.plot(time_2, right_cart_pose_2[0], '--', label='data1', linewidth=2, color='red')
plt.legend(['Demo Left', 'Demo Right', 'Correction Left', 'Correction Right'])
plt.title('Independent Demonstrations')
plt.grid()
plt.savefig("/home/oem/Desktop/interactive-bayesian-robot-learning/figure/crate_correction_x.png" , dpi=300)
# plt.show()

plt.figure()
plt.ylabel('y')
plt.xlabel('time')
plt.plot(time_1, left_cart_pose_1[1], label='data1', linewidth=2, color='black')
plt.plot(time_1, right_cart_pose_1[1], label='data1', linewidth=2, color='red')
plt.plot(time_2, left_cart_pose_2[1], '--', label='data1', linewidth=2, color='black')
plt.plot(time_2, right_cart_pose_2[1], '--', label='data1', linewidth=2, color='red')
plt.legend(['Demo Left', 'Demo Right', 'Correction Left', 'Correction Right'])
# plt.title('Independent Demonstrations')
plt.grid()
plt.savefig("/home/oem/Desktop/interactive-bayesian-robot-learning/figure/crate_correction_y.png" , dpi=300)
# plt.show()

plt.figure()
plt.ylabel('z')
plt.xlabel('time')
plt.plot(time_1, left_cart_pose_1[2], label='data1', linewidth=2, color='black')
plt.plot(time_1, right_cart_pose_1[2], label='data1', linewidth=2, color='red')
plt.plot(time_2, left_cart_pose_2[2], '--', label='data1', linewidth=2, color='black')
plt.plot(time_2, right_cart_pose_2[2], '--', label='data1', linewidth=2, color='red')
plt.legend(['Demo Left', 'Demo Right', 'Correction Left', 'Correction Right'])
plt.title('Independent Demonstrations')
plt.grid()
plt.savefig("/home/oem/Desktop/interactive-bayesian-robot-learning/figure/crate_correction_z.png" , dpi=300)
plt.show()

# plt.figure(1)
# plt.ylabel('x')
# plt.xlabel('time')
# plt.plot(time_1, left_cart_pose_1[0], label='data1', linewidth=2)
# plt.plot(time_1, right_cart_pose_1[0], label='data1', linewidth=2)

# plt.legend(['Left', 'Right'])
# plt.title('Independent Demonstrations')
# plt.grid()
# # plt.savefig("/home/oem/Desktop/interactive-bayesian-robot-learning/figure/recorded_demo_x.png" , dpi=300)
# # plt.show()

# plt.figure(2)
# plt.ylabel('y')
# plt.xlabel('time')
# plt.plot(time_1, left_cart_pose_1[1], label='data1', linewidth=2)
# plt.plot(time_1, right_cart_pose_1[1], label='data1', linewidth=2)
# plt.legend(['Left', 'Right'])
# plt.title('Independent Demonstrations')
# plt.grid()
# # plt.savefig("/home/oem/Desktop/interactive-bayesian-robot-learning/figure/recorded_demo_y.png" , dpi=300)
# # plt.show()

# plt.figure(3)
# plt.ylabel('z')
# plt.xlabel('time')
# plt.plot(time_1, left_cart_pose_1[2], label='data1', linewidth=2)
# plt.plot(time_1, right_cart_pose_1[2], label='data1', linewidth=2)
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
