function [uvms, mission] = UpdateMissionPhase(uvms, mission)
switch mission.phase
    case 1
        if norm(uvms.vTbodyGoal(1:2,4)) < 0.01
            mission.phase = 2;
            mission.phase_time = 0;
        end
    case 2
        mission.prev_action = "safe_navigation";
        mission.current_action = "landing";
        disp(uvms.altitude)
        if uvms.altitude < 0.001 && abs(uvms.theta_z) < 0.005
            mission.phase = 3;
            mission.phase_time = 0;
        end
    case 3
        mission.current_action = "landing";
        mission.current_action = "grasping";
end
end

