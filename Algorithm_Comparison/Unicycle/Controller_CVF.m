function Controller_CVF(block)
% Level-2 MATLAB file S-Function for limited integrator demo.

%   Copyright 1990-2009 The MathWorks, Inc.

setup(block);

%endfunction

function setup(block)

%% Register number of dialog parameters
block.NumDialogPrms = 2;  % 由block输入参数数目为4，输入参数为目标位置pd(2*1)，目标朝向theta_d，最小转弯半径，最大线速度，CVF设计参数k1,k2,k3

%% Register number of input and output ports
block.NumInputPorts  = 2;   %控制器输入两路，一路为当前位置p(2*1)，一路为当前姿态角theta
block.NumOutputPorts = 2;   %控制器输出两路，一路为体坐标系线速度v(2*1)，一路为角速度w

%% Setup functional port properties to dynamically
%% inherited.
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;

block.InputPort(1).Dimensions        = 2;     %当前位置p(2*1)
block.InputPort(1).DirectFeedthrough = true;
block.InputPort(2).Dimensions        = 1;     %当前姿态角theta [0,2*pi)
block.InputPort(2).DirectFeedthrough = true;

block.OutputPort(1).Dimensions       = 1;     %体坐标系线速度vx
block.OutputPort(2).Dimensions       = 1;     %体坐标系角速度w

%% Set block sample time to continuous
block.SampleTimes = [0 0];

%% Setup Dwork
block.NumContStates = 0;

%% Set the block simStateCompliance to default (i.e., same as a built-in block)
block.SimStateCompliance = 'DefaultSimState';

%% Register methods
block.RegBlockMethod('Outputs',                 @Output);

%endfunction


function Output(block)
    % 当前状态
    t = block.CurrentTime;
    global rho r1 r2 r3 v_max
    rho = 1;
    r1 = 4*rho;
    r2 = 8*rho;
    r3 = 12*rho;
    v_max = 3;

    % v_max = 1;
    p = block.InputPort(1).Data;
    theta = block.InputPort(2).Data;
    R = ang2rtm(theta);

    pd = block.DialogPrm(1).Data;
    theta_d = block.DialogPrm(2).Data;
    ed = [cos(theta_d);sin(theta_d)];
    c_p = rho;
    c_theta = pi;

    % 坐标系变换
    p_delta = pd+r2*[-ed(2);ed(1)];
    q = p-p_delta;
    
    % 体坐标系x轴参考朝向 assigned by CVF
    T = CVF(q,rho);
    theta_r = vec2ang(T);

    R_r = ang2rtm(theta_r);
    R_e = R_r' * R;
    theta_e_wegde = logm(R_e);
    d_e = norm(p-pd);
    theta_e = theta_e_wegde(2,1);

    % linear velocity control law
    % editted, took theta_e into consideration
    kv = v_max;
    vx = (0 + kv*tanh(norm(p-pd)/c_p+abs(theta_e)/c_theta));
    v = [vx;0];
    dot_p = R*v;

    % Saturated attitude control law
    w_r = omega_r(p,p_delta,dot_p);
    w0 = -kw(q,theta,rho,v)*theta_e + w_r;
    sat = min(norm(w0),1/rho*norm(v))/norm(w0);
    w = sat * w0;
    
    block.OutputPort(1).Data = vx;
    block.OutputPort(2).Data = w;

    ending_bool = vx < v_max/10 & d_e < rho/10;
    if ending_bool 
        % Display message before termination
        disp('CVF Terminating condition met. Terminating simulation...');

        % Use set_param to stop the simulation
        set_param(bdroot, 'SimulationCommand', 'stop');
    end

%endfunction

