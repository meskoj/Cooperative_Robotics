function [pandaArm] = ComputeTaskReferences(pandaArm,mission)

% Compute minimum altitude reference ALWAYS
% pandaArm.xdot.minimumAltitude = ;
% % take the smallest value, that is what matters most
% pandaArm.min_alt = min(alt_L, alt_R);

% Compute joint limits task reference ALWAYS
% Create a velocity away from the limits => move to the middle between jlmax and jlmin
% pandaArm.xdot.jointLimits = ...;

switch mission.phase
    case 1
        % Tool position and orientation task reference
        [ang, lin] = CartError(pandaArm.wTg, pandaArm.wTt);

        pandaArm.xdot.moveTool = 0.3 * [ang; lin];
        % Limits request velocities
        pandaArm.xdot.moveTool(1:3) = Saturate(pandaArm.xdot.moveTool(1:3), 1);
        pandaArm.xdot.moveTool(4:6) = Saturate(pandaArm.xdot.moveTool(4:6), 1);    
    case 2
        % Rigid Grasp Constraint
        
        % Object position and orientation task reference
        [ang, lin] = CartError();
       
        pandaArm.xdot.tool = ;
        % Limits request velocities
        pandaArm.xdot.tool(1:3) = Saturate();
        pandaArm.xdot.tool(4:6) = Saturate();

    case 3
        % Stop any motions
        % -----------------------------------------------------------------
        % Tool position and orientation task reference
        pandaArm.xdot.tool(1:3) = ...;
        pandaArm.xdot.tool(4:6) = ...;
end


