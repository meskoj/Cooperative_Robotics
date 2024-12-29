function [uvms] = ComputeActivationFunctions(uvms, mission)
% compute the activation functions here

% arm tool position control
% always active
uvms.A.t = eye(6);

switch mission.phase
    case 1
        uvms.A.altitudeControl = DecreasingBellShapedFunction(1, 2, 0, 1, uvms.altitude) * ActionTransition("AC", mission.actions.(mission.prev_action), mission.actions.(mission.current_action), mission.phase_time);
        uvms.A.vehiclePosition = eye(3) * ActionTransition("VP", mission.actions.(mission.prev_action), mission.actions.(mission.current_action), mission.phase_time);
        uvms.A.headingControl = 1 * IncreasingBellShapedFunction(0, 0.01, 0, 1, abs(uvms.theta_z)) * ActionTransition("HC", mission.actions.(mission.prev_action), mission.actions.(mission.current_action), mission.phase_time);
        uvms.A.horizontalAttitude = eye(2);
    case 2
        uvms.A.horizontalAttitude = eye(2);
        uvms.A.altitudeControl = IncreasingBellShapedFunction(0, 0.5, 0, 1, uvms.altitude) * ActionTransition("AC", mission.actions.(mission.prev_action), mission.actions.(mission.current_action), mission.phase_time);
    case 3
        uvms.A.armControl = eye(6);

end
