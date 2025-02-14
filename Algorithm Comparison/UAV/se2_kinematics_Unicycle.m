function se2_kinematics_Unicycle(block)
% Level-2 MATLAB file S-Function for limited integrator demo.

%   Copyright 1990-2009 The MathWorks, Inc.

setup(block);

%endfunction

function setup(block)

%% Register number of dialog parameters
block.NumDialogPrms = 2;  % 由block输入参数数目为2，分别为初始位置p0(2*1)与姿态theta_0 in [0,2pi)

%% Register number of input and output ports
block.NumInputPorts  = 2;   %输入两路，一路为体坐标系线速度v(2*1)，一路为体坐标系角速度Omega
block.NumOutputPorts = 2;   %输出两路，一路为当前位置p(2*1)，一路为当前姿态theta in [0,2pi)

%% Setup functional port properties to dynamically
%% inherited.
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;

block.InputPort(1).Dimensions        = 1;     %体坐标系中线速度vx
block.InputPort(1).DirectFeedthrough = false;
block.InputPort(2).Dimensions        = 1;     %体坐标系中角速度Omega
block.InputPort(2).DirectFeedthrough = false;

block.OutputPort(1).Dimensions       = 2;     % 惯性系中当前位置p
block.OutputPort(2).Dimensions       = 1; % 姿态矩阵R(:)

%% Set block sample time to continuous
block.SampleTimes = [0 0];

%% Setup Dwork
block.NumContStates = 3;        % Kinematics状态量 p 串联 theta

%% Set the block simStateCompliance to default (i.e., same as a built-in block)
block.SimStateCompliance = 'DefaultSimState';

%% Register methods
block.RegBlockMethod('InitializeConditions',    @InitConditions);
block.RegBlockMethod('Outputs',                 @Output);
block.RegBlockMethod('Derivatives',             @Derivative);

%endfunction

function InitConditions(block)

%% Initialize Dwork
    p0 = block.DialogPrm(1).Data;
    theta_0 = block.DialogPrm(2).Data;
    block.ContStates.Data = [p0;theta_0];
%endfunction

function Output(block)
    state = block.ContStates.Data;
    p = state(1:2);
    theta = mod(state(3),2*pi); % output attitude should be in [0,2pi)
    block.OutputPort(1).Data = p;
    block.OutputPort(2).Data = theta;
%endfunction

function Derivative(block)
    state = block.ContStates.Data;
    theta = state(3);
    R = ang2rtm(theta);

    vx = block.InputPort(1).Data;
    w = block.InputPort(2).Data;

    dot_p = R*[vx;0];
    block.Derivatives.Data = [dot_p;w];
%endfunction

