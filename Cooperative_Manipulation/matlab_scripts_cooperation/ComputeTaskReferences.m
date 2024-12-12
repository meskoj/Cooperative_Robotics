function [pandaArm] = ComputeTaskReferences(pandaArm,mission)

% Compute minimum altitude reference ALWAYS
pandaArm.xdot.minimumAltitude = 0.5 * (0.2 - pandaArm.wTt(3,4));

% % take the smallest value, that is what matters most
% pandaArm.min_alt = min(alt_L, alt_R);

% Compute joint limits task reference ALWAYS
% Create a velocity away from the limits => move to the middle between jlmax and jlmin
index = 1;
for jl = [pandaArm.jlmin; pandaArm.jlmax]
    if pandaArm.q(index) > jl(2) - pandaArm.delta_perc * (jl(2) - jl(1))
        pandaArm.xdot.jointLimits(index,1) = -3 * (pandaArm.q(index) - (jl(2) - pandaArm.delta_perc * (jl(2) - jl(1))));
    end
    if pandaArm.q(index) < jl(1) + pandaArm.delta_perc * (jl(2) - jl(1))
        pandaArm.xdot.jointLimits(index,1) = 3 * ((jl(1) + pandaArm.delta_perc * (jl(2) - jl(1))) - pandaArm.q(index));
    end
    index = index + 1;
end

switch mission.phase
    case 1
        %% Tool position and orientation task reference
        [ang, lin] = CartError(pandaArm.wTg, pandaArm.wTt);

        pandaArm.xdot.moveTool = 0.3 * [ang; lin];
        % Limits request velocities
        pandaArm.xdot.moveTool(1:3) = Saturate(pandaArm.xdot.moveTool(1:3), 1);
        pandaArm.xdot.moveTool(4:6) = Saturate(pandaArm.xdot.moveTool(4:6), 1);    
    case 2
        % DEBUG
        [ang, lin] = CartError(pandaArm.wTog, pandaArm.wTt);

        pandaArm.xdot.moveTool = 0.3 * [ang; lin];
        % Limits request velocities
        pandaArm.xdot.moveTool(1:3) = Saturate(pandaArm.xdot.moveTool(1:3), 1);
        pandaArm.xdot.moveTool(4:6) = Saturate(pandaArm.xdot.moveTool(4:6), 1);    
        



        % Rigid Grasp Constraint
        pandaArm.xdot.rigidConstraint = zeros(6,1);
        
        % Object position and orientation task reference
        % [ang, lin] = CartError();
       
        % pandaArm.xdot.tool = ;
        % Limits request velocities
        % pandaArm.xdot.tool(1:3) = Saturate();
        % pandaArm.xdot.tool(4:6) = Saturate();

    case 3
        % Stop any motions
        % -----------------------------------------------------------------
        % Tool position and orientation task reference
        % pandaArm.xdot.tool(1:3) = ...;
        % pandaArm.xdot.tool(4:6) = ...;
end


