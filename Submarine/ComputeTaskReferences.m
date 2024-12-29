function [uvms] = ComputeTaskReferences(uvms, mission)
% compute the task references here

switch mission.phase
    case 1
        %% Computation of vehicle distance from seafloor on world's Z-axis
        uvms.xdot.altitudeControl = 0.2 * (2 - uvms.altitude);

        %% Computation of position error
        [~, lin] = CartError(uvms.wTbodyGoal , uvms.wTv);
        uvms.xdot.vehiclePosition = 0.5 * lin;
        uvms.xdot.vehiclePosition = Saturate(uvms.xdot.vehiclePosition, 0.5);

        %% Computation of direction to goal
        versorToGoal = uvms.vTnodule(1:3,4);
        rotVec = ReducedVersorLemma([1;0;0], versorToGoal);
        uvms.theta_z = rotVec(3);
        % The minus in the parenthesys comes from the rotation matrix
        % The minus in front for the control
        disp(rotVec)
        uvms.xdot.headingControl = 0.7 * uvms.theta_z;
        uvms.xdot.headingControl = Saturate(uvms.xdot.headingControl, 0.8);

        %% Computation of horizontal attitude
        % TODO reducedVersorLemma
        ang = VersorLemma(uvms.vTw(1:3,1:3), eye(3))*(-1);
        ang = ang(1:2);
        uvms.xdot.horizontalAttitude = 0.7 * ang;
        uvms.xdot.horizontalAttitude = Saturate(uvms.xdot.horizontalAttitude, 0.8);
    case 2
        uvms.xdot.altitudeControl = 0.8 * -uvms.altitude;
        uvms.xdot.altitudeControl = Saturate(uvms.xdot.altitudeControl, 1);

        %% Computation of horizontal attitude
        ang = VersorLemma(uvms.vTw(1:3,1:3), eye(3))*(-1);
        ang = ang(1:2);
        uvms.xdot.horizontalAttitude = 0.7 * ang;
        uvms.xdot.horizontalAttitude = Saturate(uvms.xdot.horizontalAttitude, 0.8);
    case 3
        uvms.xdot.vehiclePosition = zeros(3,1);
        [ang, lin] = CartError(uvms.wTg, uvms.wTt);
        ang = uvms.vTw(1:3,1:3) * ang;
        lin = uvms.vTw(1:3,1:3) * lin;
        uvms.xdot.armControl = 0.7 * [ang;lin];
end
end
