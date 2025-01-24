function [uvms] = InitUVMS(robotname)

% uvms.vTb
% transformation matrix betwene the arm base wrt vehicle frame
% expresses how the base of the arm is attached to the vehicle
% do NOT change, since it must be coherent with the visualization tool
if (strcmp(robotname, 'DexROV'))
    % do NOT change
    uvms.vTb = [rotation(pi, 0, pi) [0.167 0 -0.43]'; 0 0 0 1];
else
    if (strcmp(robotname, 'Robust'))
        % do NOT change
        uvms.vTb = [rotation(0, 0, pi) [0.85 0 -0.42]'; 0 0 0 1];
    end
end

uvms.q_dot = [0 0 0 0 0 0 0]';
uvms.p_dot = [0 0 0 0 0 0]';

% joint limits corresponding to the actual MARIS arm configuration
uvms.jlmin  = [-2.9;-1.6;-2.9;-2.95;-2.9;-1.65;-2.8];
uvms.jlmax  = [2.9;1.65;2.9;0.01;2.9;1.25;2.8];

% to be computed at each time step
uvms.wTv = eye(4,4);
uvms.wTt = eye(4,4);
uvms.vTw = eye(4,4);
uvms.vTe = eye(4,4);
uvms.vTt = eye(4,4);
uvms.vTg = eye(4,4);
uvms.Ste = eye(6,6);
uvms.bTe = eye(4,4);
uvms.bJe = eye(6,7);
uvms.djdq = zeros(6,7,7);
uvms.mu  = 0;
uvms.phi = zeros(3,1);
uvms.sensorDistance = 0;
uvms.wTnodule = zeros(4);

uvms.Jt_a = [];
uvms.Jt_v = [];
uvms.Jt = [];

uvms.xdot.altitudeControl = 0;
uvms.xdot.horizontalAttitude = zeros(2);
uvms.xdot.vehiclePosition = zeros(3);
uvms.xdot.headingControl = 0;
uvms.xdot.armControl = zeros(6,1);
uvms.xdot.noMovement = zeros(6,1);

uvms.A.altitudeControl = 0;
uvms.A.horizontalAttitude = zeros(2);
uvms.A.vehiclePosition = zeros(3);
uvms.A.headingControl = 0;
uvms.A.armControl = zeros(6);
uvms.A.noMovement = zeros(6);

uvms.J.altitudeControl = zeros(1,13);
uvms.J.horizontalAttitude = zeros(2,13);
uvms.J.vehiclePosition = zeros(3,13);
uvms.J.headingControl = zeros(1,13);
uvms.J.armControl = zeros(6,13);
uvms.J.noMovement = zeros(6,13);

uvms.horizontalMisalignmentVector = zeros(2, 1);
uvms.r_tn = zeros(3, 1);
uvms.altitude = 0;

uvms.transitionTimes = [];

end

