# %%
import matplotlib.pyplot as plt
import numpy as np
import pathlib

from recording_node import Recorder
def load(filename):
    data = np.load(str(pathlib.Path().resolve()) + '/recordings_final/' + str(filename) + '.npz')
    return data


# %%
filename = "19_12_36_58"
data = load(filename=filename)
time = np.linspace(0, data['recorded_cart_pose'].shape[1], data['recorded_cart_pose'].shape[1])/100

'cart_pose: x, y, z, qz, qy, qz, qw'
right_cart_pose = data["recorded_cart_pose"][:7]
left_cart_pose = data['recorded_cart_pose'][7:]

right_attractor_pose = data['recorded_attractor_pose'][:7]
left_attractor_pose = data['recorded_attractor_pose'][7:]

right_attractor_stiffness = data['recorded_stiffness'][:7]
left_attractor_stiffness = data['recorded_stiffness'][7:]
# %%
plt.figure(1)
plt.ylabel('x')
plt.plot(time, left_cart_pose[0], label='data1')
# plt.legend('')
plt.title('Left cartesian position')

plt.figure(2)
plt.ylabel('y')
plt.plot(time, left_cart_pose[1], label='data1')
# plt.legend()
plt.title('Left cartesian position')

plt.figure(3)
plt.ylabel('z')
plt.plot(time, left_cart_pose[2], label='data1')
# plt.legend()
plt.title('Left cartesian position')

plt.figure(4)
plt.ylabel('x')
plt.plot(time, right_cart_pose[0], label='data1')
# plt.legend()
plt.title('right cartesian position')

plt.figure(5)
plt.ylabel('y')
plt.plot(time, right_cart_pose[1], label='data1')
# plt.legend()
plt.title('right cartesian position')

plt.figure(6)
plt.ylabel('z')
plt.plot(time, right_cart_pose[2], label='data1')
# plt.legend()
plt.title('right cartesian position')

# %%
right_err = (right_cart_pose1[:3] - right_attractor_pose1[:3])
left_err = (left_cart_pose1[:3] - left_attractor_pose1[:3])

plt.figure(4)
plt.ylabel('x')
plt.plot(right_err[0]-left_err[0])
plt.legend()

plt.figure(5)
plt.ylabel('y')
plt.plot(right_err[1]-left_err[1])
plt.legend()

plt.figure(6)
plt.ylabel('z')
plt.plot(right_err[2]-left_err[2])
plt.legend()

plt.figure(7)

# %%

plt.figure(1)
plt.ylabel('x')
plt.plot(time, left_cart_pose1[0], label='data1', color='r')
plt.plot(data2_t, left_cart_pose2[0], label='data2')
plt.legend()

plt.figure(2)
plt.ylabel('y')
plt.plot(time, left_cart_pose1[1], label='data1', color='r')
plt.plot(data2_t, left_cart_pose2[1], label='data2')
plt.legend()

plt.figure(3)
plt.ylabel('z')
plt.plot(time, left_cart_pose1[3], label='data1', color='r')
plt.plot(data2_t, left_cart_pose2[3], label='data2')
plt.legend()
# %%
plt.figure(1)
plt.plot(time, left_cart_pose1[0], color='red', label='With perturbation')
plt.plot(data2_t, left_cart_pose2[0], label='Without perturbation')
plt.legend()
# %%

error_based_force_right = data1['recorded_stiffness'][:3]*(data1['recorded_cart_pose'][:3]-data1['recorded_attractor_pose'][:3])
error_based_force_left = data1['recorded_stiffness'][7:10]*(data1['recorded_cart_pose'][7:10]-data1['recorded_attractor_pose'][7:10])

plt.figure(1)
plt.plot(error_based_force_left[0])

plt.figure(2)
plt.plot(data1['recorded_force_torque'][6])
# %%

plt.figure(1)
plt.plot(time, data1["recorded_stiffness"][7])
plt.plot(data2_t, data2["recorded_stiffness"][7])

plt.figure(2)
plt.plot(time, data1["recorded_stiffness"][8])
plt.plot(data2_t, data2["recorded_stiffness"][8])

plt.figure(3)
plt.plot(time, data1["recorded_stiffness"][9])
plt.plot(data2_t, data2["recorded_stiffness"][9])

# %%
fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, sharex=True, figsize=(10,10))

ax1.plot(time, data1["recorded_stiffness"][8], label='recorded stiffness no correction', color='r')
ax1.plot(data2_t, data2["recorded_stiffness"][8], label='recorded stiffness with correction', color='b')
ax2.plot(data2_t, data2["recorded_feedback"][4], label='correction input', color='k')
ax3.plot(data2_t, data2["recorded_cart_pose"][8]-data2["recorded_attractor_pose"][8], label='distance to attractor', color='k')
ax4.plot(data2_t, data2['recorded_force_torque'][7], label='externally applied force', color='k')

ax1.set_ylabel('Stiffness (N/m)')
ax2.set_ylabel('Magnitude')
ax3.set_ylabel('Distance (m)')
ax4.set_ylabel('Force (N)')

ax1.set_ylim(550,900)
ax2.set_ylim(-1, 0.6)
ax1.legend()
ax2.legend()
ax3.legend()
ax4.legend()

# %%
