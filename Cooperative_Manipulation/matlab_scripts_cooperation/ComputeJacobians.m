function [pandaArm] = ComputeJacobians(pandaArm,mission)
% compute the relevant Jacobians here
% joint limits
% tool-frame position control (to do)
% initial arm posture ( [0.0167305, -0.762614, -0.0207622, -2.34352, -0.0305686, 1.53975, 0.753872] ) 
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

% Jacobian from base to end-effector
pandaArm.bJe = geometricJacobian(pandaArm.franka,[pandaArm.q',0,0],'panda_link7');
% THE JACOBIAN bJe has dimension 6x9 (the matlab model include the joint
% of the gripper). YOU MUST RESIZE THE MATRIX IN ORDER TO CONTROL ONLY THE
% 7 JOINTS OF THE ROBOTIC ARM. 
pandaArm.bJe = pandaArm.bJe(:, 1:7);

eSt = [eye(3) zeros(3);
    skew(pandaArm.wTb(1:3,1:3) * pandaArm.bTe(1:3,1:3) * pandaArm.eTt(1:3,4))', eye(3)]; 
wRb = pandaArm.wTb(1:3,1:3);
wRb_jacobian = [wRb zeros(3); zeros(3) wRb];
pandaArm.wJt  = eSt * wRb_jacobian * pandaArm.bJe;

%% JOINT LIMITS
pandaArm.J.jointLimits = eye(7);

%% MINIMUM ALTITUDE
pandaArm.J.minimumAltitude = pandaArm.wJt(6,:);

%% MOVE TOOL
switch mission.phase
    case 1 
        pandaArm.J.moveTool = pandaArm.wJt;
    
    case 2
        pandaArm.wJo = pandaArm.tSo * pandaArm.wJt;
        pandaArm.J.moveTool = pandaArm.wJo;

        pandaArm.H = pandaArm.wJo * pinv(pandaArm.wJo);
end
end
