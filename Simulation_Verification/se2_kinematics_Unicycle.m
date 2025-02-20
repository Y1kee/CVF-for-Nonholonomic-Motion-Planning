function se2_kinematics_Unicycle(block)

setup(block);

%endfunction

function setup(block)

%% Register number of dialog parameters
block.NumDialogPrms = 2;

%% Register number of input and output ports
block.NumInputPorts  = 2;
block.NumOutputPorts = 2;

%% Setup functional port properties to dynamically
%% inherited.
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;

block.InputPort(1).Dimensions        = 1;
block.InputPort(1).DirectFeedthrough = false;
block.InputPort(2).Dimensions        = 1;
block.InputPort(2).DirectFeedthrough = false;

block.OutputPort(1).Dimensions       = 2;
block.OutputPort(2).Dimensions       = 1;

%% Set block sample time to continuous
block.SampleTimes = [0 0];

%% Setup Dwork
block.NumContStates = 3;

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
    theta = mod(state(3),2*pi);
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

