# KinovaGen3-impedance-controller
A simple joint impedance controller applied on 7 Dof Kinova Gen3 robot arm and a Supernumerary Robot Limbs(SRLs) model.

## Install
Here is the code for installing testing in Windows 10.

* **Downloading the codes**
```
git clone https://github.com/CROBOT974/Development-of-Trajectory-Based-Control-of-Supernumerary-Robotic-Limbs-for-Assembling-Tasks.git
```
* **Installing the required packages**

The inverse kinematics function relies on [ikpy](https://github.com/Phylliade/ikpy), and the [fcl-python](https://github.com/BerkeleyAutomation/python-fcl/tree/master) library is referred for collision detection. 
```
pip install numpy
pip install mujoco==3.2.7
```
