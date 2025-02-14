# Curvature-Constrained Vector Field for Motion Planning of Nonholonomic Robots
 
This research focuses on navigating the 2D nonholonomic robot featuring an upper bound on the trajectory curvature to the target configuration via the curvature-constrained vector field (CVF) and the saturated control law with dynamic gain. 

The preprint is available on: xxxxx.

In this repository, we provide the source codes for the Simulation Verification (corresponding to Section V.A in the paper), Algorithm Comparison (Section V.B), Experiments on Ackermann UGV (Section VI.A) and Hardware-In-the-Loop (HIL) Experiments on Fixed-Wing UAV (Section V.B).

## Simulation Verification
The simulation is based on MATLAB Simulink.
Run the first simulation example in Section V.A with "RUN_Example.m", all the parameters and outputs of simulation are with comments.

## Algorithm Comparison
The comparative study is based on MATLAB Simulink and is conducted on unicycle and UAV, respectively. 
Based on the unicycle, the proposed method (Simulink model Unicycle_CVF.slx) is compared with the DVF (He, TAC, 2024, Simulink model "Unicycle_DVF.slx") and AVF (Panagou, TAC, 2017, Simulink model "Unicycle_AVF.slx").
Based on the fixed-wing UAV, the proposed method (Simulink model UAV_CVF.slx) is compared with the CLVF (Pothen, JGCD, 2017, Simulink model "UAV_CLVF.slx") and GVF (Yao, TRO, 2021, Simulink model "UAV_GVF.slx").

## Experiments on Ackermann UGV
In the experiments, we utilize the ROS Toolboc provided by MathWorks to connect our Simulink model ".slx" with the ROS noetic, sending control inputs to the LIMO ackermann vehicle by AgileX.
For reference, we provide the Gazebo simulation, where the motion planning algorithm ".m" and Simulink model ".slx" are identical to the ones we used in the experiments.

## HIL Experiments on Fixed-Wing UAV
In the HIL experiments, we extend the proposed method to generate feasible setpoints for the 3D fixed-wing UAV, namely the reference attitude and thrust.
For reference, we provide the Gazebo simulation, where the motion planning algorithm ".cpp" is identical to the one we used in the experiments.