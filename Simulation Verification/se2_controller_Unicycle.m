function se2_controller_Unicycle(block)

setup(block);

%endfunction

function setup(block)

%% Register number of dialog parameters
block.NumDialogPrms = 7;

%% Register number of input and output ports
block.NumInputPorts  = 2; 
block.NumOutputPorts = 5;

%% Setup functional port properties to dynamically
%% inherited.
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;

block.InputPort(1).Dimensions        = 2;
block.InputPort(1).DirectFeedthrough = true;
block.InputPort(2).Dimensions        = 1;
block.InputPort(2).DirectFeedthrough = true;

block.OutputPort(1).Dimensions       = 1;
block.OutputPort(2).Dimensions       = 1;
block.OutputPort(3).Dimensions       = 1;
block.OutputPort(4).Dimensions       = 1;
block.OutputPort(5).Dimensions       = 1;

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
    % current configuration
    p = block.InputPort(1).Data;
    theta = block.InputPort(2).Data;
    R = ang2rtm(theta);
    % parameters for controller
    pd = block.DialogPrm(1).Data;
    theta_d = block.DialogPrm(2).Data;
    rho = block.DialogPrm(3).Data;
    v_max = block.DialogPrm(4).Data;
    k1 = block.DialogPrm(5).Data;
    k2 = block.DialogPrm(6).Data;
    k3 = block.DialogPrm(7).Data;
    global r1 r2 r3
    r1 = k1*rho;
    r2 = k2*rho;
    r3 = k3*rho;
    
    % reference orientation assigned by CVF
    p_delta = pd+r2*[-sin(theta_d);cos(theta_d)];
    q = p-p_delta;
    T = CVF(q,rho);
    theta_r = vec2ang(T);
    
    % orientation error
    R_r = ang2rtm(theta_r);
    R_e = R_r' * R;
    theta_e_wegde = logm(R_e);
    d_e = norm(p-pd);
    theta_e = theta_e_wegde(2,1);

    % linear velocity control law
    c_p = 12*rho;
    c_theta = pi; 
    kv = v_max;
    vx = (0 + kv*tanh(norm(p-pd)/c_p+abs(theta_e)/c_theta));
    v = [vx;0];
    dot_p = R*v;

    % Saturated attitude control law
    w_r = omega_r(p,p_delta,dot_p);
    w0 = -kw(q,theta,rho,v)*theta_e + w_r;
    sat = min(norm(w0),1/rho*norm(v))/norm(w0);
    w = sat * w0;
    
    % control inputs
    block.OutputPort(1).Data = vx;
    block.OutputPort(2).Data = w;
    block.OutputPort(3).Data = kw(q,theta,rho,v);
    block.OutputPort(4).Data = theta_r;
    block.OutputPort(5).Data = w0;

    ending_bool = vx < v_max/10 & d_e < rho/2;
    if ending_bool 
        % Display message before termination
        disp('Terminating condition met. Terminating simulation...');

        % Use set_param to stop the simulation
        set_param(bdroot, 'SimulationCommand', 'stop');
    end

%endfunction

