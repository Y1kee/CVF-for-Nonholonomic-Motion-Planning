# Curvature-Constrained Vector Field for Motion Planning of Nonholonomic Robots

This research focuses on navigating the 2D nonholonomic robot featuring an upper bound on the trajectory curvature to the target configuration via the curvature-constrained vector field (CVF) and the saturated control law with dynamic gain.

The preprint is available on: xxxxx.

In this repository, we provide the source codes for the Simulation Verification (corresponding to Section V.A in the paper), Algorithm Comparison (Section V.B), Experiments on Ackermann UGV (Section VI.A) and Hardware-In-the-Loop (HIL) Experiments on Fixed-Wing UAV (Section V.B).

## Simulation Verification

The simulation is based on MATLAB Simulink.
Run the first simulation example in Section V.A with "RUN_Example.m", all the parameters and outputs of simulation are with comments.

## Algorithm Comparison

The comparative study is based on MATLAB Simulink and is conducted on unicycle and UAV, respectively.
Based on the unicycle, the proposed method (Simulink model Unicycle_CVF.slx) is compared with the [DVF](10.1109/TAC.2024.3406475) (Unicycle_DVF.slx) and [AVF](10.1109/TAC.2016.2576020) (Unicycle_AVF.slx).
Based on the fixed-wing UAV, the proposed method (UAV_CVF.slx) is compared with the [CLVF](https://doi.org/10.2514/1.G002281) (UAV_CLVF.slx) and [GVF](10.1109/TRO.2020.3043690) (UAV_GVF.slx).

## Experiments on Ackermann UGV

In the experiments, we utilize the ROS Toolbox provided by MathWorks to connect our Simulink model "Ackermann_UGV_exp.slx" with the ROS noetic, sending control inputs to the [LIMO by AgileX](https://global.agilex.ai/products/limo-pro) (in the Ackermann mode).
In the experiments, the Simulink model receives ROS topic '/odom' to get the vehicle's configuration and sends ROS topic '/cmd_vel' as the control inputs of the vehicle.
The model can be easily adjusted and integrated into your experiment environment for replicate the results and extend the research.

## HIL Experiments on Fixed-Wing UAV

In the HIL experiments, we extend the proposed method to generate feasible setpoints for the 3D fixed-wing UAV, namely the reference attitude and thrust.
For reference, we provide the Gazebo simulation demo below, where the motion planning algorithm "sacontrol_utils.cpp" is identical to the one we used in the HIL experiments.

---

### Quickstart

Here are instructions to perform fixed-wing simulation in Gazebo Classic via PX4.

#### Dependency

* Ubuntu 20.04
* ROS Noetic
* PX4 and its ROS Wrappers

  * [Installation of PX4 and MAVROS](https://docs.px4.io/main/en/ros/mavros_installation.html)
  * [Launch Gazebo Classic with ROS Wrappers](https://docs.px4.io/main/en/simulation/ros_interface.html#launching-gazebo-classic-with-ros-wrappers)

#### Gazebo Simulation

##### Workspace

```bash
cd ~ 
mkdir -p ws_cvf/src && cd ws_cvf/src
git clone https://github.com/Y1kee/CVF-for-Nonholonomic-Motion-Planning
git clone https://github.com/ros-simulation/gazebo_ros_pkgs  # Gazebo-ROS Toolbox 
cd .. && catkin_make
```

##### Launch Simulation

```bash
cd <PX4-Autopilot_clone>
source ~/ws_cvf/devel/setup.bash
source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic

roslaunch cvf_fix_wing sitl_startall.launch
```
