function [uvms, mission] = UpdateMissionPhase(uvms, mission)
switch mission.phase
    case 1
        if norm(uvms.vTbodyGoal(1:2,4)) < 0.01
            mission.phase = 2;
            mission.phase_time = 0;
            mission.current_action = "landing";
        end
    case 2
        mission.prev_action = "landing";
        if uvms.altitude < 0.001
            mission.phase = 3;
            mission.phase_time = 0;
            mission.current_action = "grasping";
        end
    case 3
        mission.prev_action = "grasping";
end
end

