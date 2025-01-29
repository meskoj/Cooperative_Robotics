function [pandaArms, mission] = UpdateMissionPhase(pandaArms, mission)
switch mission.phase
    case 1  %Go To Grasping Points
        [ang, lin] = CartError(pandaArms.ArmL.wTg, pandaArms.ArmL.wTt);
        [ang1, lin1] = CartError(pandaArms.ArmR.wTg, pandaArms.ArmR.wTt);
        if norm(ang) < 0.01 && norm(lin) < 0.01 && norm(ang1) < 0.01 && norm(lin1) < 0.01
            mission.phase = 2;
            mission.phase_time = 0;
            pandaArms.transitionTimes(1,1) = mission.wall_time;
            mission.prev_action = "go_to";
            mission.current_action = "coop_manip";
        end

    case 2 % Cooperative Manipulation Start
        [ang, lin] = CartError(pandaArms.ArmL.wTog, pandaArms.ArmL.wTnt);

        if norm(ang) < 0.01 && norm(lin) < 0.01
            pandaArms.transitionTimes(1,2) = mission.wall_time;
            mission.phase = 3;
            mission.phase_time = 0;
            mission.prev_action = "coop_manip";
            mission.current_action = "end_motion";
        end

    case 3 % Finish motion

end
end

