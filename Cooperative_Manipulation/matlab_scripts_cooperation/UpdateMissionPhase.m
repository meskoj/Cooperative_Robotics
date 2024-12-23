function [pandaArmL, pandaArmR, mission] = UpdateMissionPhase(pandaArmL, pandaArmR, mission)
switch mission.phase
    case 1  %Go To Grasping Points
        % computing the errors for the go-to action defining tasks

        % max error: 1/10 cm and 1deg
        [angL,linL] = CartError(pandaArmL.wTg, pandaArmL.wTt);
        [angR,linR] = CartError(pandaArmR.wTg, pandaArmR.wTt);
        % disp([norm(angL) ,norm(angR), norm(linL), norm(linR)])
        if norm(angL) < 0.01 && norm(angR) < 0.01 && norm(linL) < 0.01 && norm(linR) < 0.01
            mission.current_action = "coop_manip";
            mission.phase_time = 0;
            mission.phase = 2;

            % Distance between tools to object
            t_r_to_left = pinv(pandaArmL.wTt) * [pandaArmL.wTo(1:3,4);1];
            w_r_to_left = pandaArmL.wTt(1:3,1:3) * t_r_to_left(1:3,1);
            pandaArmL.r_to = w_r_to_left;
            t_r_to_right = pinv(pandaArmR.wTt) * [pandaArmR.wTo(1:3,4);1];
            w_r_to_right = pandaArmR.wTt(1:3,1:3) * t_r_to_right(1:3,1);
            pandaArmR.r_to = w_r_to_right;
            disp([w_r_to_left, pandaArmL.wTt(1:3,1:3)' * (pandaArmL.wTo(1:3,4) - pandaArmL.wTt(1:3,4))])

            % Rigid body jacobians
            pandaArmL.tSo = [eye(3) zeros(3);
                skew(pandaArmL.r_to)', eye(3)];

            pandaArmR.tSo = [eye(3) zeros(3);
                skew(pandaArmR.r_to)', eye(3)];

            pandaArmL.tTnt = [eye(3), t_r_to_left(1:3); 0 0 0 1];
            pandaArmR.tTnt = [eye(3), t_r_to_right(1:3); 0 0 0 1];
        end

    case 2 % Cooperative Manipulation Start
        % computing the errors for the rigid move-to task

        mission.prev_action = "coop_manip";
        % max error: 1 cm and 3deg
        [angL,linL] = CartError(pandaArmL.wTog, pandaArmL.wTt);
        [angR,linR] = CartError(pandaArmR.wTog, pandaArmR.wTt);
        if norm(angL) < deg2rad(3) && norm(angR) < deg2rad(3) && norm(linL) < 0.01 && norm(linR) < 0.01
            mission.phase = 3;
            mission.phase_time = 0;
        end
    case 3 % Finish motion

end

