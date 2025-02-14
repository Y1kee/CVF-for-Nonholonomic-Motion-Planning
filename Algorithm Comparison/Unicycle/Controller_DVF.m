function Controller_DVF(block)
% Level-2 MATLAB file S-Function for limited integrator demo.

%   Copyright 1990-2009 The MathWorks, Inc.

setup(block);

%endfunction

function setup(block)

%% Register number of dialog parameters
block.NumDialogPrms = 2;  % 由block输入参数数目为4，输入参数为目标位置pd(2*1)，目标朝向theta_d

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
    global v_max rho
    % 当前状态
    t = block.CurrentTime;


    p = block.InputPort(1).Data;
    theta = block.InputPort(2).Data;

    pd = block.DialogPrm(1).Data;
    theta_d = block.DialogPrm(2).Data;

    k_v = 0.1;
    k_w = 0.1;
    k_a = 1;

    g=[cos(theta) -sin(theta) p(1);
        sin(theta)  cos(theta) p(2);
        0          0         1];
    gd=[cos(theta_d) -sin(theta_d) pd(1);
        sin(theta_d)  cos(theta_d) pd(2);
        0          0         1];
    I=eye(3);

    X=logm(gd\g/I);
    theta_e=X(2,1);
    Vgb=X(1:2,3);

    vx = - k_v * Vgb(1);
    vx = max(min(vx,v_max),-v_max);
    w = -k_w*theta_e + k_a*atan(Vgb(2)/Vgb(1));
    

    block.OutputPort(1).Data = vx;
    block.OutputPort(2).Data = w;
    % block.OutputPort(2).Data = min(max(w,-1/rho/vx),1/rho/vx);
    
    d_e = norm(p-pd);
    ending_bool = abs(vx) < v_max/10 & d_e < rho/10;
    if ending_bool 
        % Display message before termination
        disp('DVF Terminating condition met. Terminating simulation...');

        % Use set_param to stop the simulation
        set_param(bdroot, 'SimulationCommand', 'stop');
    end

%endfunction

