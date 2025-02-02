function [uvms] = ComputeActivationFunctions(uvms, mission)
% compute the activation functions here

    uvms.A.t = eye(6);
    switch mission.phase
        case 1
            uvms.A.altitudeControlSafety = DecreasingBellShapedFunction(1, 2, 0, 1, uvms.altitude) * ActionTransition("ACS", mission.actions.(mission.prev_action), mission.actions.(mission.current_action), mission.phase_time);
            uvms.A.horizontalAttitude = eye(2) * IncreasingBellShapedFunction(deg2rad(1), deg2rad(3), 0, 1, max(abs(uvms.horizontalMisalignmentVector))) * ActionTransition("HA", mission.actions.(mission.prev_action), mission.actions.(mission.current_action), mission.phase_time);
            uvms.A.vehiclePosition = eye(3) * ActionTransition("VP", mission.actions.(mission.prev_action), mission.actions.(mission.current_action), mission.phase_time);
            uvms.A.headingControl = ActionTransition("HC", mission.actions.(mission.prev_action), mission.actions.(mission.current_action), mission.phase_time);

        case 2
            uvms.A.altitudeControlSafety = DecreasingBellShapedFunction(1, 2, 0, 1, uvms.altitude) * ActionTransition("ACS", mission.actions.(mission.prev_action), mission.actions.(mission.current_action), mission.phase_time);
            uvms.A.altitudeControlAD = ActionTransition("ACAD", mission.actions.(mission.prev_action), mission.actions.(mission.current_action), mission.phase_time);
            uvms.A.horizontalAttitude = eye(2) * IncreasingBellShapedFunction(deg2rad(1), deg2rad(3), 0, 1, max(abs(uvms.horizontalMisalignmentVector))) * ActionTransition("HA", mission.actions.(mission.prev_action), mission.actions.(mission.current_action), mission.phase_time);
            uvms.A.vehiclePosition = eye(3) * ActionTransition("VP", mission.actions.(mission.prev_action), mission.actions.(mission.current_action), mission.phase_time);
            uvms.A.vehiclePositionXY = eye(2) * ActionTransition("VPXY", mission.actions.(mission.prev_action), mission.actions.(mission.current_action), mission.phase_time);
            uvms.A.headingControl = ActionTransition("HC", mission.actions.(mission.prev_action), mission.actions.(mission.current_action), mission.phase_time);

        case 3
            uvms.A.altitudeControlAD = ActionTransition("ACAD", mission.actions.(mission.prev_action), mission.actions.(mission.current_action), mission.phase_time);
            uvms.A.horizontalAttitude = eye(2) * IncreasingBellShapedFunction(deg2rad(1), deg2rad(3), 0, 1, max(abs(uvms.horizontalMisalignmentVector))) * ActionTransition("HA", mission.actions.(mission.prev_action), mission.actions.(mission.current_action), mission.phase_time);
            uvms.A.vehiclePositionXY = eye(2) * ActionTransition("VPXY", mission.actions.(mission.prev_action), mission.actions.(mission.current_action), mission.phase_time);
            uvms.A.headingControl = ActionTransition("HC", mission.actions.(mission.prev_action), mission.actions.(mission.current_action), mission.phase_time);
            uvms.A.armControl = eye(6) * ActionTransition("AM", mission.actions.(mission.prev_action), mission.actions.(mission.current_action), mission.phase_time);
            uvms.A.noMovement = eye(6) * ActionTransition("NM", mission.actions.(mission.prev_action), mission.actions.(mission.current_action), mission.phase_time);
    end
end
