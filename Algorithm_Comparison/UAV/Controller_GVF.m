function Controller_GVF(block)
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
    global rho v_max r2
    kw = 1;
    % v_max = 1;
    p = block.InputPort(1).Data;
    theta = block.InputPort(2).Data;
    R = ang2rtm(theta);

    pd = block.DialogPrm(1).Data;
    theta_d = block.DialogPrm(2).Data;
    ed = [cos(theta_d);sin(theta_d)];
    d_e = norm(p-pd);
    

    % 坐标系变换
    p_delta = pd+r2*[-ed(2);ed(1)];
    q = p-p_delta;
    % d_e = abs(norm(p-p_delta)-r2);  % distance error to the limit cycle
    
    % Linear velocity
    vx = v_max;
    v = [vx;0];
    dot_p = R*v;
    
    % Angular velocity
    T = GVF(q);    % GVF向量场确定Rd第1列（T）已归一化
    dot_T = dGVFdt(q,dot_p);    % T向量场各分量（已归一化）对时间求导
    w_r = [-T(2), T(1)]*dot_T;
    
    Rd=[T(1),-T(2); T(2),T(1)];   % 此刻需要跟踪的姿态Ra
    theta_e_hat = logm(Rd'*R);
    theta_e = theta_e_hat(2,1);

    w = -kw*sin(theta_e) + w_r;     % from Yao, 2021, TRO, Eq.24a
    
    block.OutputPort(1).Data = vx;
    block.OutputPort(2).Data = w;
    % block.OutputPort(2).Data = min(max(w,-1/rho/vx),1/rho/vx);

    ending_bool =  d_e < rho/10;
    if ending_bool 
        % Display message before termination
        disp('GVF Terminating condition met. Terminating simulation...');

        % Use set_param to stop the simulation
        set_param(bdroot, 'SimulationCommand', 'stop');
    end


    

%endfunction

