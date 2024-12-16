function [pandaArms] = ComputeTaskReferences(pandaArms,mission)

% Compute distance between tools for plotting
pandaArms.dist_tools = norm(pandaArms.ArmL.wTt(1:3, 4) - pandaArms.ArmR.wTt(1:3, 4));

% Compute minimum altitude reference ALWAYS
pandaArms.ArmL.xdot.minimumAltitude = 0.5 * (0.2 - pandaArms.ArmL.wTt(3,4));
pandaArms.ArmR.xdot.minimumAltitude = 0.5 * (0.2 - pandaArms.ArmR.wTt(3,4));

% Compute joint limits task reference ALWAYS
% Create a velocity away from the limits => move to the middle between jlmax and jlmin
index = 1;
for jl = [pandaArms.jlmin; pandaArms.jlmax]
    if pandaArms.ArmL.q(index) > jl(2) - pandaArms.delta_perc * (jl(2) - jl(1))
        pandaArms.ArmL.xdot.jointLimits(index,1) = -3 * (pandaArms.ArmL.q(index) - (jl(2) - pandaArms.delta_perc * (jl(2) - jl(1))));
    end
    if pandaArms.ArmL.q(index) < jl(1) + pandaArms.delta_perc * (jl(2) - jl(1))
        pandaArms.ArmL.xdot.jointLimits(index,1) = 3 * ((jl(1) + pandaArms.delta_perc * (jl(2) - jl(1))) - pandaArms.ArmL.q(index));
    end

    if pandaArms.ArmR.q(index) > jl(2) - pandaArms.delta_perc * (jl(2) - jl(1))
        pandaArms.ArmR.xdot.jointLimits(index,1) = -3 * (pandaArms.ArmR.q(index) - (jl(2) - pandaArms.delta_perc * (jl(2) - jl(1))));
    end
    if pandaArms.ArmR.q(index) < jl(1) + pandaArms.delta_perc * (jl(2) - jl(1))
        pandaArms.ArmR.xdot.jointLimits(index,1) = 3 * ((jl(1) + pandaArms.delta_perc * (jl(2) - jl(1))) - pandaArms.ArmR.q(index));
    end
    index = index + 1;
end

switch mission.phase
    case 1
        %% LEFT ARM
        % -----------------------------------------------------------------
        % Tool position and orientation task reference
% e.g. CartError(wTg, wTv) returns the error that makes <v> -> <g> 
        [ang, lin] = CartError(pandaArms.ArmL.wTg, pandaArms.ArmL.wTt);

        pandaArms.ArmL.xdot.pose = 0.5 * [ang; lin];
            % limit the requested velocities...
        pandaArms.ArmL.xdot.pose(1:3) = Saturate(pandaArms.ArmL.xdot.pose(1:3), 1);
        pandaArms.ArmL.xdot.pose(4:6) = Saturate(pandaArms.ArmL.xdot.pose(4:6), 1);

        %% RIGHT ARM
        % -----------------------------------------------------------------
        % Tool position and orientation task reference
        [ang, lin] = CartError(pandaArms.ArmR.wTg, pandaArms.ArmR.wTt);

        pandaArms.ArmR.xdot.pose = 0.5 * [ang; lin];
        % limit the requested velocities...
        pandaArms.ArmR.xdot.pose(1:3) = Saturate(pandaArms.ArmR.xdot.pose(1:3), 1);

        pandaArms.ArmR.xdot.pose(4:6) = Saturate(pandaArms.ArmR.xdot.pose(4:6), 1);

    case 2
        %% LEFT ARM
        % -----------------------------------------------------------------
        % Object position and orientation task reference
        tTnt_left = [eye(3) pandaArms.ArmL.wTt(1:3,1:3)' * pandaArms.ArmL.r_to; 0 0 0 1];
        pandaArms.ArmL.wTnt = pandaArms.ArmL.wTt * tTnt_left;
        [ang, lin] = CartError(pandaArms.ArmL.wTog, pandaArms.ArmL.wTnt);

        pandaArms.ArmL.xdot.bimanualPose = 0.5 * [ang; lin]; % limit the requested velocities...
        % pandaArms.ArmL.xdot.bimanualPose = 0.1 * [0.0;0.0;0.5;0.0;0.0;0]; % limit the requested velocities...
        pandaArms.ArmL.xdot.bimanualPose(1:3) = Saturate(pandaArms.ArmL.xdot.bimanualPose(1:3), 1);
        pandaArms.ArmL.xdot.bimanualPose(4:6) = Saturate(pandaArms.ArmL.xdot.bimanualPose(4:6), 1);

        %% RIGHT ARM
        % -----------------------------------------------------------------
        % Object position and orientation task reference
        tTnt_right = [eye(3) pandaArms.ArmR.wTt(1:3,1:3)' * pandaArms.ArmR.r_to; 0 0 0 1];
        pandaArms.ArmR.wTnt = pandaArms.ArmR.wTt * tTnt_right;
        [ang, lin] = CartError(pandaArms.ArmR.wTog, pandaArms.ArmR.wTnt);

        pandaArms.ArmR.xdot.bimanualPose = 0.5 * [ang; lin]; % limit the requested velocities...
        % pandaArms.ArmR.xdot.bimanualPose = 0.1 * [0.0;0.0;0.5;0.0;0.0;0]; % limit the requested velocities...
        pandaArms.ArmR.xdot.bimanualPose(1:3) = Saturate(pandaArms.ArmR.xdot.bimanualPose(1:3), 1);
        pandaArms.ArmR.xdot.bimanualPose(4:6) = Saturate(pandaArms.ArmR.xdot.bimanualPose(4:6), 1);

        pandaArms.ArmL.xdot.rigidConstraint = zeros(6,1);
        pandaArms.ArmR.xdot.rigidConstraint = zeros(6,1);

    case 3
        %% Stop any motions
        % LEFT ARM
        % -----------------------------------------------------------------
        pandaArms.ArmL.xdot.stopMotors = zeros(7,1);

        %% RIGHT ARM
        % -----------------------------------------------------------------
        pandaArms.ArmR.xdot.stopMotors = zeros(7,1);
end
