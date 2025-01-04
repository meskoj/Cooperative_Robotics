function [uvms] = ComputeActivationFunctions(uvms, mission)
% compute the activation functions here

% arm tool position control
% always active
uvms.A.t = eye(6);
    % disp([max(abs(uvms.horizontalMisalignmentVector(1:2))), uvms.horizontalMisalignmentVector']);
switch mission.phase
    case 1
        uvms.A.altitudeControl = DecreasingBellShapedFunction(1, 2, 0, 1, uvms.altitude) * ActionTransition("AC", mission.actions.(mission.prev_action), mission.actions.(mission.current_action), mission.phase_time);
        uvms.A.horizontalAttitude = eye(2) * IncreasingBellShapedFunction(deg2rad(1), deg2rad(3), 0, 1, max(abs(uvms.horizontalMisalignmentVector))) * ActionTransition("HA", mission.actions.(mission.prev_action), mission.actions.(mission.current_action), mission.phase_time);
        uvms.A.vehiclePosition = eye(3) * ActionTransition("VP", mission.actions.(mission.prev_action), mission.actions.(mission.current_action), mission.phase_time);
        uvms.A.headingControl = ActionTransition("HC", mission.actions.(mission.prev_action), mission.actions.(mission.current_action), mission.phase_time);
    case 2
        uvms.A.altitudeControl = ActionTransition("AC", mission.actions.(mission.prev_action), mission.actions.(mission.current_action), mission.phase_time);
        uvms.A.horizontalAttitude = eye(2) * IncreasingBellShapedFunction(deg2rad(1), deg2rad(3), 0, 1, max(abs(uvms.horizontalMisalignmentVector))) * ActionTransition("HA", mission.actions.(mission.prev_action), mission.actions.(mission.current_action), mission.phase_time);
        uvms.A.headingControl = ActionTransition("HC", mission.actions.(mission.prev_action), mission.actions.(mission.current_action), mission.phase_time);
    case 3
        %% DEACTIVATING TASKS
        uvms.A.vehiclePosition = eye(3) * ActionTransition("VP", mission.actions.(mission.prev_action), mission.actions.(mission.current_action), mission.phase_time);
        uvms.A.horizontalAttitude = eye(2) * IncreasingBellShapedFunction(deg2rad(1), deg2rad(3), 0, 1, max(abs(uvms.horizontalMisalignmentVector))) * ActionTransition("HA", mission.actions.(mission.prev_action), mission.actions.(mission.current_action), mission.phase_time);
        uvms.A.altitudeControl = ActionTransition("AC", mission.actions.(mission.prev_action), mission.actions.(mission.current_action), mission.phase_time);
        uvms.A.headingControl = ActionTransition("HC", mission.actions.(mission.prev_action), mission.actions.(mission.current_action), mission.phase_time);

        %% ACTIVE TASKS
        uvms.A.armControl = eye(6) * ActionTransition("AM", mission.actions.(mission.prev_action), mission.actions.(mission.current_action), mission.phase_time);
        uvms.A.noMovement = eye(6) * ActionTransition("NM", mission.actions.(mission.prev_action), mission.actions.(mission.current_action), mission.phase_time);


end
