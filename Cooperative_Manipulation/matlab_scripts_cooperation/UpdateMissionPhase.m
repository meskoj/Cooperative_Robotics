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
                end
                
            case 2 % Cooperative Manipulation Start 
                % computing the errors for the rigid move-to task

                mission.prev_action = "coop_manip";
                % max error: 1 cm and 3deg
               
            case 3 % Finish motion
                
end

