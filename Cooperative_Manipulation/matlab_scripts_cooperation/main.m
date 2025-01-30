addpath('./simulation_scripts');
clc;
clear;
close all
real_robot = false;
%% Initialization - DON'T CHANGE ANYTHING from HERE ...
% Simulation variables (integration and final time)
deltat = 0.005;
end_time = 20;
loop = 1;
maxloops = ceil(end_time/deltat);
mission.phase = 1;
mission.phase_time = 0;
model = load("panda.mat");

% UDP Connection with Franka Interface
if real_robot == true
    hudprLeft = dsp.UDPReceiver('LocalIPPort',1501,'MaximumMessageLength',255);
    hudprRight = dsp.UDPReceiver('LocalIPPort',1503,'MaximumMessageLength',255);
    hudpsLeft = dsp.UDPSender('RemoteIPPort',1500);
    hudpsLeft.RemoteIPAddress = '127.0.0.1';
    hudpsRight = dsp.UDPSender('RemoteIPPort',1502);
    hudpsRight.RemoteIPAddress = '127.0.0.1';
else
    hudps = dsp.UDPSender('RemoteIPPort',1505);
    hudps.RemoteIPAddress = '127.0.0.1';
end
%% TO HERE

% Init robot model
wTb_left = eye(4); %fixed transformation word -> base left
wRb_right = rotation(0,0,pi);
wpb_right = [1.06;-0.01;0];
wTb_right = [wRb_right, wpb_right; 0 0 0 1]; % fixed transformation word -> base2
pandaArmL = InitRobot(model,wTb_left);
pandaArmR = InitRobot(model,wTb_right);

% Preallocation
plt = InitDataPlot(maxloops);

% Init object frame
obj_length = 0.06;
w_obj_pos = [0.5 0 0.30]';
w_obj_ori = rotation(0,0,0);
pandaArmL.wTo = [w_obj_ori w_obj_pos; 0 0 0 1];
pandaArmR.wTo = [w_obj_ori w_obj_pos; 0 0 0 1];

theta = -44.9949;% FIXED ANGLE BETWEEN EE AND TOOL
tool_length = 0.2124;% FIXED DISTANCE BETWEEN EE AND TOOL
% Define trasnformation matrix from ee to tool.
pandaArmL.eTt = [rotation(0, 0, deg2rad(theta)), [0;0;tool_length]; 0 0 0 1];
pandaArmR.eTt = [rotation(0, 0, deg2rad(theta)), [0;0;tool_length]; 0 0 0 1];

% Transformation matrix from <t> to <w>
pandaArmL.wTt = pandaArmL.wTe * pandaArmL.eTt;
pandaArmR.wTt = pandaArmR.wTe * pandaArmR.eTt;

%% Defines the goal position for the end-effector/tool position task
% First goal reach the grasping points.
pandaArmL.wTg = [pandaArmL.wTt(1:3,1:3) * rotation(0, deg2rad(20), 0), [w_obj_pos - [obj_length; 0; 0] / 2]; 0 0 0 1]; % Rotation of 30 degrees around y axis from goal to tool
pandaArmR.wTg = [pandaArmR.wTt(1:3,1:3) * rotation(0, deg2rad(20), 0), [w_obj_pos + [obj_length; 0; 0] / 2]; 0 0 0 1];
%% Second goal move the object
%% DEBUG
pandaArmL.wTog = [pandaArmL.wTt(1:3,1:3) * rotation(0.0, deg2rad(20), 0.0), [0.60 0.40 0.48]'; 0 0 0 1]; % Rotation of 30 degrees around y axis from goal to tool
pandaArmR.wTog = [pandaArmR.wTt(1:3,1:3) * rotation(0.0, deg2rad(20), 0.0), [0.60 0.40 0.48]'; 0 0 0 1];

% pandaArmL.wTog = [rotation(1.0, 0.0, deg2rad(20)) * pandaArmL.wTt(1:3,1:3) * rotation(0.0, deg2rad(20), 0.0), [0.60 0.40 0.48]'; 0 0 0 1]; % Rotation of 30 degrees around y axis from goal to tool
% pandaArmR.wTog = [rotation(1.0, 0.0, deg2rad(20)) * pandaArmR.wTt(1:3,1:3) * rotation(0.0, deg2rad(20), 0.0), [0.60 0.40 0.48]'; 0 0 0 1];

% pandaArmL.wTog = [pandaArmL.wTt(1:3,1:3) * rotation(0.0, deg2rad(20), 0.0), [1.0 0.00 0.38]'; 0 0 0 1]; % Rotation of 30 degrees around y axis from goal to tool
% pandaArmR.wTog = [pandaArmR.wTt(1:3,1:3) * rotation(0.0, deg2rad(20), 0.0), [1.0 0.00 0.38]'; 0 0 0 1];

%% G2
% pandaArmL.wTog = [pandaArmL.wTt(1:3,1:3) * rotation(0.0, deg2rad(20), 0.0), [0.4 0 0]'; 0 0 0 1]; % Rotation of 30 degrees around y axis from goal to tool
% pandaArmR.wTog = [pandaArmR.wTt(1:3,1:3) * rotation(0.0, deg2rad(20), 0.0), [0.4 0 0]'; 0 0 0 1];

% pandaArmL.wTog = [eye(3), [0.60 0.40 0.48]'; 0 0 0 1]; % Rotation of 30 degrees around y axis from goal to tool
% pandaArmR.wTog = [eye(3), [0.60 0.40 0.48]'; 0 0 0 1];

%% Mission configuration

mission.prev_action = "go_to";
mission.current_action = "go_to";

mission.phase = 1;
mission.phase_time = 0;
mission.wall_time = 0;
mission.transitionTimes = [];
% Define the active tasks for each phase of the mission
% T = move tool task
% JL = joint limits task
% MA = minimum altitude task
% RC = rigid constraint task
mission.actions.go_to.tasks = ["T", "MA", "JL"];
mission.actions.coop_manip.tasks = ["T", "JL", "MA", "RC"];
mission.actions.end_motion.tasks = ["MA"];

mu_l = 0.5;
mu_r = 0.5;

%% CONTROL LOOP
for t = 0:deltat:end_time

    % Receive UDP packets - DO NOT EDIT
    if real_robot == true
        dataLeft = step(hudprLeft);
        dataRight = step(hudprRight);
        % wait for data (to decide)
        if t == 0
            while(isempty(dataLeft))
                dataLeft = step(hudprLeft);
                pause(deltat);
            end
            while(isempty(dataRight))
                dataRight = step(hudprRight);
                pause(deltat);
            end
        end
        qL = typecast(dataLeft, 'double');
        qR = typecast(dataRight, 'double');
        pandaArmL.q = qL;
        pandaArmR.q = qR;
    end

    % update all the involved variables
    [pandaArmL] = UpdateTransforms(pandaArmL,mission);
    [pandaArmR] = UpdateTransforms(pandaArmR,mission);
    [pandaArmL] = ComputeJacobians(pandaArmL,mission);
    [pandaArmR] = ComputeJacobians(pandaArmR,mission);
    [pandaArmL] = ComputeActivationFunctions(pandaArmL,mission);
    [pandaArmR] = ComputeActivationFunctions(pandaArmR,mission);
    [pandaArmL] = ComputeTaskReferences(pandaArmL,mission);
    [pandaArmR] = ComputeTaskReferences(pandaArmR,mission);


    % main kinematic algorithm initialization
    % ydotbarL order is [qdot_1, qdot_2, ..., qdot_7, xdot, ydot, zdot, omega_x, omega_y, omega_z]
    % the vector of the vehicle linear and angular velocities are assumed
    % projected on <v>

    ydotbarL = zeros(7,1);
    QpL = eye(7);
    ydotbarR = zeros(7,1);
    QpR = eye(7);

    % Used by the Move-To task
    tool_jacobian_L = zeros(6, 7);
    tool_jacobian_R = zeros(6, 7);
    if (mission.phase == 1)
        % In this phase the tool frame coincide with the center of the
        % gripper
        tool_jacobian_L = pandaArmL.wJt;
        tool_jacobian_R = pandaArmR.wJt;
    elseif(mission.phase == 2)
        % In this phase the tool frame coincide with the object frame
        tool_jacobian_L = pandaArmL.wJo;
        tool_jacobian_R = pandaArmR.wJo;
    end


    % ADD minimum distance from table
    % add all the other tasks here!
    % the sequence of iCAT_task calls defines the priority

    % First Manipulator TPIK (left)
    % Task: Tool Move-To
    [QpL, ydotbarL] = iCAT_task(pandaArmL.A.stopAll, eye(7), QpL, ydotbarL, pandaArmL.xdot.stopAll, 0.0001,   0.01, 10);
    [QpL, ydotbarL] = iCAT_task(pandaArmL.A.jointLimits, pandaArmL.J.jointLimits, QpL, ydotbarL, pandaArmL.xdot.jointLimits, 0.0001,   0.01, 10);
    [QpL, ydotbarL] = iCAT_task(pandaArmL.A.minimumAltitude, pandaArmL.J.minimumAltitude, QpL, ydotbarL, pandaArmL.xdot.minimumAltitude, 0.0001,   0.01, 10);
    [QpL, ydotbarL] = iCAT_task(pandaArmL.A.moveTool, pandaArmL.J.moveTool, QpL, ydotbarL, pandaArmL.xdot.moveTool, 0.0001,   0.01, 10);

    % Second manipulator TPIK (right)
    % Task: Tool Move-To
    [QpR, ydotbarR] = iCAT_task(pandaArmR.A.stopAll, eye(7), QpR, ydotbarR, pandaArmR.xdot.stopAll, 0.0001,   0.01, 10);
    [QpR, ydotbarR] = iCAT_task(pandaArmR.A.jointLimits, pandaArmR.J.jointLimits, QpR, ydotbarR, pandaArmR.xdot.jointLimits, 0.0001,   0.01, 10);
    [QpR, ydotbarR] = iCAT_task(pandaArmR.A.minimumAltitude, pandaArmR.J.minimumAltitude, QpR, ydotbarR, pandaArmR.xdot.minimumAltitude, 0.0001,   0.01, 10);
    % if mission.phase == 1
        [QpR, ydotbarR] = iCAT_task(pandaArmR.A.moveTool, pandaArmR.J.moveTool, QpR, ydotbarR, pandaArmR.xdot.moveTool, 0.0001,   0.01, 10);
    % else
    %     [QpR, ydotbarR] = iCAT_task(pandaArmR.A.moveTool, pandaArmR.J.moveTool, QpR, ydotbarR, pandaArmL.xdot.moveTool, 0.0001,   0.01, 10);
    % end

    % COOPERATION hierarchy
    % SAVE THE NON COOPERATIVE VELOCITIES COMPUTED
    % disp([pandaArmL.xdot.moveTool, pandaArmR.xdot.moveTool]);
    xtl = tool_jacobian_L * ydotbarL;
    xtr = tool_jacobian_R * ydotbarR;

    mu_0 = 0.001;
    mu_l = mu_0 + norm(pandaArmL.xdot.moveTool - xtl);
    mu_r = mu_0 + norm(pandaArmL.xdot.moveTool - xtr);
    % disp([mu_l, mu_r]);

    coop_vel = (1 / (mu_l + mu_r)) * (mu_l * xtl + mu_r * xtr);

    C = [pandaArmL.H -pandaArmR.H];

    x_tab = [pandaArmL.H zeros(6);
        zeros(6) pandaArmR.H] * (eye(12)  -pinv(C) * C) * [coop_vel; coop_vel];


    coopVelL = x_tab(1:6);
    coopVelR = x_tab(7:12);

    pandaArmL.coopVel = coopVelL;
    pandaArmL.nonCoopVel = xtl;

    pandaArmR.coopVel = coopVelR;
    pandaArmR.nonCoopVel = xtr;

    % Task: Left Arm Cooperation
    % ...

    if mission.phase == 2
        ydotbarL = zeros(7,1);
        QpL = eye(7);
        [QpL, ydotbarL] = iCAT_task(pandaArmL.A.moveToolWithConstraint, pandaArmL.J.moveTool, QpL, ydotbarL, coopVelL, 0.0001,   0.01, 10);
        [QpL, ydotbarL] = iCAT_task(pandaArmL.A.jointLimits, pandaArmL.J.jointLimits, QpL, ydotbarL, pandaArmL.xdot.jointLimits, 0.0001,   0.01, 10);
        [QpL, ydotbarL] = iCAT_task(pandaArmL.A.minimumAltitude, pandaArmL.J.minimumAltitude, QpL, ydotbarL, pandaArmL.xdot.minimumAltitude, 0.0001,   0.01, 10);
    end

    % this task should be the last one
    [QpL, ydotbarL] = iCAT_task(eye(7),...
        eye(7),...
        QpL, ydotbarL,...
        zeros(7,1),...
        0.0001,   0.01, 10);
    % Task: Right Arm Cooperation
    % ...

    if mission.phase == 2
        ydotbarR = zeros(7,1);
        QpR = eye(7);
        [QpR, ydotbarR] = iCAT_task(pandaArmR.A.moveToolWithConstraint, pandaArmR.J.moveTool, QpR, ydotbarR, coopVelR, 0.0001,   0.01, 10);
        [QpR, ydotbarR] = iCAT_task(pandaArmR.A.jointLimits, pandaArmR.J.jointLimits, QpR, ydotbarR, pandaArmR.xdot.jointLimits, 0.0001,   0.01, 10);
        [QpR, ydotbarR] = iCAT_task(pandaArmR.A.minimumAltitude, pandaArmR.J.minimumAltitude, QpR, ydotbarR, pandaArmR.xdot.minimumAltitude, 0.0001,   0.01, 10);
    end
    % this task should be the last one
    [QpR, ydotbarR] = iCAT_task(eye(7),...
        eye(7),....
        QpR, ydotbarR,...
        zeros(7,1),...
        0.0001,   0.01, 10);
    % get the two variables for integration
    pandaArmL.q_dot = ydotbarL(1:7);
    pandaArmR.q_dot = ydotbarR(1:7);

    pandaArmL.x = tool_jacobian_L * pandaArmL.q_dot;
    pandaArmR.x = tool_jacobian_R * pandaArmR.q_dot;

    % Integration
    pandaArmL.q = pandaArmL.q(1:7) + pandaArmL.q_dot*deltat;
    pandaArmR.q = pandaArmR.q(1:7) + pandaArmR.q_dot*deltat;

    %Send udp packets [q_dot1, ..., q_dot7] DO NOT CHANGE
    if real_robot == false
        pandaArmL.q = pandaArmL.q(1:7) + pandaArmL.q_dot*deltat;
        pandaArmR.q = pandaArmR.q(1:7) + pandaArmR.q_dot*deltat;
    end
    %Send udp packets [q_dot1, ..., q_dot7]
    if real_robot == true
        step(hudpsLeft,[t;pandaArmL.q_dot]);
        step(hudpsRight,[t;pandaArmR.q_dot]);
    else
        step(hudps,[pandaArmL.q',pandaArmR.q'])
    end

    % check if the mission phase should be changed
    mission.phase_time = mission.phase_time + deltat;
    mission.wall_time = mission.wall_time + deltat;
    [pandaArmL,pandaArmR,mission] = UpdateMissionPhase(pandaArmL,pandaArmR,mission);

    % Compute distance between tools for plotting
    pandaArmL.dist_tools = norm(pandaArmL.wTt(1:3, 4) - pandaArmR.wTt(1:3, 4));

    % Update data for plots
    plt = UpdateDataPlot(plt,pandaArmL,pandaArmR,t,loop, mission);

    loop = loop + 1;
    % add debug prints here
    if (mod(t,0.1) == 0)
        t
        mission.phase
    end

    % enable this to have the simulation approximately evolving like real
    % time. Remove to go as fast as possible
    % WARNING: MUST BE ENABLED IF CONTROLLING REAL ROBOT !
    SlowdownToRealtime(deltat);

end
PrintPlot(plt);
