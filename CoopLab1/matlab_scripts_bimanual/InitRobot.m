function [pandaArms] = InitRobot(model,wTb_left,wTb_right)

%% DO NOT CHANGE FROM HERE ...
% Init two field of the main structure pandaArms containing the two robot
% model
pandaArms.ArmL = model;
pandaArms.ArmR = model;
% Init robot basic informations (q_init, transformation matrices ...)
pandaArms.ArmL.q = [0.0167305,-0.762614,-0.0207622,-2.34352,-0.0305686,1.53975,0.753872]';%check rigid body tree DOCUMENTATION
pandaArms.ArmR.q = pandaArms.ArmL.q;
pandaArms.ArmL.q_dot = [0 0 0 0 0 0 0]';
pandaArms.ArmR.q_dot = [0 0 0 0 0 0 0]';
pandaArms.ArmL.bTe = getTransform(pandaArms.ArmL.franka,[pandaArms.ArmL.q',0,0],'panda_link7');
pandaArms.ArmR.bTe = getTransform(pandaArms.ArmR.franka,[pandaArms.ArmR.q',0,0],'panda_link7');
pandaArms.ArmL.wTb = wTb_left;
pandaArms.ArmR.wTb = wTb_right;
pandaArms.ArmL.wTe = pandaArms.ArmL.wTb*pandaArms.ArmL.bTe;
pandaArms.ArmR.wTe = pandaArms.ArmR.wTb*pandaArms.ArmR.bTe;

% joint limits corresponding to the actual Panda by Franka arm configuration
jlmin = [-2.8973;-1.7628;-2.8973;-3.0718;-2.8973;-0.0175;-2.8973];
jlmax = [2.8973;1.7628;2.8973;-0.0698;2.8973;3.7525;2.8973];

% Init relevance Jacobians
pandaArms.ArmL.bJe = eye(6,7);
pandaArms.ArmR.bJe = eye(6,7);
pandaArms.Jjl = [];

%% ... TO HERE
% Init Task Reference vectors

% Init Activation function for activate or deactivate tasks




end

