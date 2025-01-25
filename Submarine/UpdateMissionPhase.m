function [uvms, mission] = UpdateMissionPhase(uvms, mission)
    switch mission.phase
        case 1
            if norm(uvms.w_r_gv(1:2)) < 0.01 && abs(rad2deg(uvms.theta_z)) < 1
                mission.phase = 2;
                mission.phase_time = 0;
                uvms.transitionTimes(1) = uvms.t;
                mission.prev_action = "safe_navigation";
                mission.current_action = "landing";
            end
        case 2
            if uvms.altitude < 0.001
                mission.phase = 3;
                mission.phase_time = 0;
                uvms.transitionTimes(2) = uvms.t;
                mission.prev_action = "landing";
                mission.current_action = "grasping";
            end
    end
end

