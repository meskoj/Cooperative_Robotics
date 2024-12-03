function [pandaArms, mission] = UpdateMissionPhase(pandaArms, mission)    
        switch mission.phase
            case 1  %Go To Grasping Points
                [ang, lin] = CartError(pandaArms.ArmL.wTg, pandaArms.ArmL.wTt);
                [ang1, lin1] = CartError(pandaArms.ArmR.wTg, pandaArms.ArmR.wTt);
                if norm(ang) < 0.001 && norm(lin) < 0.001 && norm(ang1) < 0.001 && norm(lin1) < 0.001
                    mission.phase = 2;
                end
                
            case 2 % Cooperative Manipulation Start 
                [ang, lin] = CartError(pandaArms.ArmL.wTog, pandaArms.ArmL.wTnt);

                if norm(ang) < 0.01 && norm(lin) < 0.01
                    mission.phase = 3;
                end
                
            case 3 % Finish motion
                
        end
end

