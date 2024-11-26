function [pandaArms] = ComputeJacobians(pandaArms,mission)
% compute the relevant Jacobians here
% joint limits
% tool-frame position control (to do)
% initial arm posture ( [0.0167305, -0.762614, -0.0207622, -2.34352,
% -0.0305686, 1.53975, 0.753872] )
%
% remember: the control vector is:
% [q_dot]
% [qdot_1, qdot_2, ..., qdot_7]
%
% therefore all task jacobians should be of dimensions
% m x 14
% where m is the row dimension of the task, and of its reference rate

% computation for tool-frame Jacobian
% [omegax_t omegay_t omegaz_t xdot_t ydot_t zdot_t] = Jt ydot
% [angular velocities; linear velocities]

% Left Arm base to ee Jacobian
pandaArms.ArmL.bJe = geometricJacobian(pandaArms.ArmL.franka, ...
    [pandaArms.ArmL.q',0,0],'panda_link7');%DO NOT EDIT
% Right Arm base to ee Jacobian
pandaArms.ArmR.bJe = geometricJacobian(pandaArms.ArmR.franka, ...
    [pandaArms.ArmR.q',0,0],'panda_link7');%DO NOT EDIT

pandaArms.ArmL.bJe = pandaArms.ArmL.bJe(:, 1:7);
pandaArms.ArmR.bJe = pandaArms.ArmR.bJe(:, 1:7);

% Top three rows are angular velocities, bottom three linear velocities
eSt_left = [eye(3) zeros(3);
    skew(pandaArms.ArmL.eTt(1:3,4))', eye(3)];
eSt_right = [eye(3) zeros(3);
    skew(pandaArms.ArmR.eTt(1:3,4))', eye(3)];

wRb = pandaArms.ArmR.wTb(1:3,1:3);
wRb_j = [wRb zeros(3);
    zeros(3) wRb];

% eSt_left = eye(6);
% eSt_right = eye(6);

pandaArms.ArmL.wJt  = eSt_left * pandaArms.ArmL.bJe;
pandaArms.ArmR.wJt  = eSt_right * wRb_j * pandaArms.ArmR.bJe;

pandaArms.ArmL.J.minimumAltitude = pandaArms.ArmL.wJt(6, :);
pandaArms.ArmR.J.minimumAltitude = pandaArms.ArmR.wJt(6, :);

pandaArms.ArmL.J.pose = pandaArms.ArmL.wJt;
pandaArms.ArmR.J.pose = pandaArms.ArmR.wJt;

if (mission.phase == 2)
    % pandaArms.ArmL.wJo = ...; TODO
    % pandaArms.ArmR.wJo = ...;

    % Common Jacobians
    % pandaArms.Jjl = ...; TODO

end
