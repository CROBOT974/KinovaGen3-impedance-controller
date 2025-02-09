# KinovaGen3-impedance-controller
A simple joint impedance controller was applied to the 7 Dof Kinova Gen3 robot arm and a model of supernumerary robot limbs (SRLs).

## Install
Here is the code for installing testing in Windows 10.

* **Downloading the codes**
```
git clone https://github.com/CROBOT974/KinovaGen3-impedance-controller.git
```
* **Installing the required packages**
```
pip install numpy
pip install mujoco==3.2.7
```
## Run a demo
* **Test the whole scenario**
```
python -m tests.test_Gen3
python -m tests.test_SRLs
```
* **demo of the scenario**
