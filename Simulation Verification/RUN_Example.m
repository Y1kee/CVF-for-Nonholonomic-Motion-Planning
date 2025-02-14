%% This script runs the first simulation example in Section V.A
% Initialization
clear
clc
close all

%% Parameter Specification
rho = 1;                                                                    % minimal turning radius of the vehicle
k_1 = 4; k_2 = 8; k_3 = 12;                                                 % design parameters for CVF, where r_i = k_i*rho
p_0 = [0;0.5];                                                              % initial position
theta_0 = 5*pi/4;                                                           % initial orientation
p_d = [4;4*sqrt(3)];                                                        % target position
theta_d = 5*pi/6;                                                           % target orientation
v_max = rho;                                                                % maximum linear velocity, while the minimum is zero as default

modelName = 'Controlled_Ackermann';
load_system(modelName);
set_param(modelName, 'MaxStep', '0.1');
set_param([modelName, '/Kinematics'], 'p_0', mat2str(p_0), 'theta_0', mat2str(theta_0));
set_param([modelName, '/Controller'], 'p_d', mat2str(p_d), 'theta_d', mat2str(theta_d),...
    'rho', mat2str(rho), 'v_max', mat2str(v_max), ...
    'k1', mat2str(k_1), 'k2', mat2str(k_2), 'k3', mat2str(k_3));


%% Run the simulation and output the results
simout = sim(modelName);
t = simout.timeout;                                                         % simulation time steps
theta = simout.theta;                                                       % orientation
theta_r = simout.theta_r;                                                   % reference orientation
p = simout.p;                                                               % position
kappa = simout.kappa;                                                       % trajectory curvature
omega = simout.omega;                                                       % angular velocity with saturation
omega_0 = simout.omega_0;                                                   % angular velocity without saturation
vx = simout.vx;                                                             % linear velocity
kw = simout.kw;                                                             % dynamic gain
