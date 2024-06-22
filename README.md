# SIMPLe: Safe, Interactive Movement Primitives Learning

This is the code used for the experiments done for the paper: 
### Interactive Imitation Learning of Bimanual Movement Primitives 

published in Transaction of Mechatronics (T-MECH).

- [The official repository](https://github.com/franzesegiovanni/SIMPLe.git)

- [The paper](https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=10215052)

### What is SIMPLe? 

SIMPLe stands for Safe, Interactive Movement Primitives Learning. It is a framework that allows a robot to learn a movement primitive from a human demonstration. The motion is represensted as a trajectory or a graph of nodes. The encoding uses both the position and the time of each node of the trajectory. The algorithm uses both the information of the time and the currect position of the robot to attract the robot to most correlated nodes in the trajectory. 

### Installation 

This is a ROS package. So you need to add it to your ROS workspace and catkin build it. 

Clone the repository in your ROS workspace. 

```bash
git clone https://github.com/franzesegiovanni/SIMPLe.git 
```

Build your workspace. 

```bash
catkin build
```

### How to run SIMPLe on a single robot 

Install the human friently impedance controller
- [The human-friendly impedance controller](https://github.com/franzesegiovanni/franka_human_friendly_controllers.git)

You can use the SIMPLE/main as example to start recording and executing motion using the SIMPLe trajectory representation. 

To run an example directly on the robot, you can use the the script main_single_arm.py. 

I suggest to open it in vscode and after installing the jupyter extension, you can run it cell by cell.

If you want to run it in simulation, please follow the instructions in human friendly controller on how to run the controller in simulation.
### How to run SIMPLe bimanual

Install the bimanual controller in the your workspace. Follow the instruction in the link
- [The bimanual impedance controller](https://github.com/franzesegiovanni/franka_bimanual_controllers)

Add this repo to your workspace and install it. 

In the directory scripts there are many examples that where used in the development of the paper for interactive learning of bimanual manipulation.


### Cite us

If you find this repository useful, please cite: 

```
Franzese, G., de Souza Rosa, L., Verburg, T., Peternel, L. and Kober, J., 2023. Interactive imitation learning of bimanual movement primitives. IEEE/ASME Transactions on Mechatronics.
```
