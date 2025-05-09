function [pandaArm] = InitRobot(model,wTb)

%% DO NOT CHANGE FROM HERE ...
% Init two field of the main structure pandaArm containing the two robot
% model
pandaArm = model;
% Init robot basic informations (q_init, transformation matrices ...)
pandaArm.q = [0.0167305,-0.762614,-0.0207622,-2.34352,-0.0305686,1.53975,0.753872]';%check rigid body tree DOCUMENTATION
pandaArm.q_dot = [0 0 0 0 0 0 0]';
pandaArm.alt = 0.20;

pandaArm.bTe = getTransform(pandaArm.franka,[pandaArm.q',0,0],'panda_link7');
pandaArm.wTb = wTb;
pandaArm.wTe = pandaArm.wTb*pandaArm.bTe;

% joint limits corresponding to the actual Panda by Franka arm configuration
pandaArm.jlmin = [-2.8973;-1.7628;-2.8973;-3.0718;-2.8973;-0.0175;-2.8973]';
pandaArm.jlmax = [2.8973;1.7628;2.8973;-0.0698;2.8973;3.7525;2.8973]';

% Init relevance Jacobians
pandaArm.bJe = eye(6,7);
pandaArm.Jjl = [];

%% Initialize all the needed values
% MINIMUM ALTITUDE
pandaArm.A.minimumAltitude = 0;
pandaArm.xdot.minimumAltitude = 0;
pandaArm.J.minimumAltitude = zeros(1,7);

% JOINT LIMITS
pandaArm.delta_perc = 0.1;
pandaArm.A.jointLimits = zeros(7);
pandaArm.xdot.jointLimits = zeros(7,1);
pandaArm.J.jointLimits = zeros(7);

% MOVE TOOL
pandaArm.A.moveTool = zeros(6);
pandaArm.xdot.moveTool = zeros(6,1);
pandaArm.J.moveTool = zeros(6,7);

% STOP ALL
pandaArm.A.stopAll = zeros(7);
pandaArm.xdot.stopAll = zeros(7,1);
pandaArm.J.stopAll = zeros(7);


pandaArm.r_to = [];
pandaArm.tSo = [];
pandaArm.H = eye(6);
end

