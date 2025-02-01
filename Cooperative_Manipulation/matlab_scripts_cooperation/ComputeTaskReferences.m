function [pandaArm] = ComputeTaskReferences(pandaArm,mission)
%% MINIMUM ALTITUDE
% Compute minimum altitude reference ALWAYS
pandaArm.xdot.minimumAltitude = 0.5 * (0.2 - pandaArm.wTt(3,4));

%% JOINT LIMITS
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

%% MOVE TOOL
switch mission.phase
    case 1
        [ang, lin] = CartError(pandaArm.wTg, pandaArm.wTt);

        pandaArm.xdot.moveTool = 0.3 * [ang; lin];
        % Limits request velocities
        pandaArm.xdot.moveTool(1:3) = Saturate(pandaArm.xdot.moveTool(1:3), 1);
        pandaArm.xdot.moveTool(4:6) = Saturate(pandaArm.xdot.moveTool(4:6), 1);    
    case 2
        [ang, lin] = CartError(pandaArm.wTog, pandaArm.wTt); %w Tt is modified in UpdateTransforms, 
                                                             % so that the tool position is the correct one

        pandaArm.xdot.moveTool = 0.3 * [ang; lin];
        % Limits request velocities
        pandaArm.xdot.moveTool(1:3) = Saturate(pandaArm.xdot.moveTool(1:3), 1);
        pandaArm.xdot.moveTool(4:6) = Saturate(pandaArm.xdot.moveTool(4:6), 1);    
        
    case 3
        % Stop any motions
        pandaArm.xdot.stopAll = zeros(7,1);
end


