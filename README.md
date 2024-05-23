# SIMPLe: Safe, Interactive Movement Primitives Learning

This is the code used for the experiments done for the paper: 
### Interactive Imitation Learning of Bimanual Movement Primitives 

published in Transaction of Mechatronics (T-MECH).

- [The official repository](https://github.com/franzesegiovanni/SIMPLe.git)

- [The paper](https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=10215052)


### Installation 

Clone the repository in your ROS workspace. 

```bash
git clone https://github.com/franzesegiovanni/SIMPLe.git 
```

Build your workspace. 

```bash
catkin build
```

### How to run SIMPLe bimanual

Install the bimanual controller in the your workspace. Follow the instruction in the link
- [The bimanual impedance controller](https://github.com/franzesegiovanni/franka_bimanual_controllers)

Add this repo to your workspace and install it. 

In the directory scripts there are many examples. 



### How to run SIMPLe on a single robot 

Install the human friently impedance controller
- [The human-friendly impedance controller](https://github.com/franzesegiovanni/franka_human_friendly_controllers.git)

You can use the SIMPLE/main as example to start recording and executing motion using the SIMPLe trajectory representation. 

### Cite us

If you find this repository useful, please cite: 

```
Franzese, G., de Souza Rosa, L., Verburg, T., Peternel, L. and Kober, J., 2023. Interactive imitation learning of bimanual movement primitives. IEEE/ASME Transactions on Mechatronics.
```
