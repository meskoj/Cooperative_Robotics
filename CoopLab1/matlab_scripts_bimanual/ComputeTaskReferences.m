function [pandaArms] = ComputeTaskReferences(pandaArms,mission)
% Compute distance between tools for plotting
pandaArms.dist_tools = norm(pandaArms.ArmL.wTt(1:3, 4) - pandaArms.ArmR.wTt(1:3, 4));
% Compute minimum altitude reference ALWAYS

pandaArms.ArmL.xdot.minimumAltitude = 0.2 * (0.2 - pandaArms.ArmL.wTt(3,4));
pandaArms.ArmR.xdot.minimumAltitude = 0.2 * (0.2 - pandaArms.ArmR.wTt(3,4));

% Compute joint limits task reference ALWAYS
% Create a velocity away from the limits => move to the middle between jlmax and jlmin

%pandaArms.ArmL.xdot.jl = ...;
%pandaArms.ArmR.xdot.jl = ...;

switch mission.phase
    case {1, 2}
        % LEFT ARM
        % -----------------------------------------------------------------
        % Tool position and orientation task reference
% e.g. CartError(wTg, wTv) returns the error that makes <v> -> <g> 
        [ang, lin] = CartError(pandaArms.ArmL.wTg, pandaArms.ArmL.wTt);

        pandaArms.ArmL.xdot.pose = 0.2 * [ang; lin];
            % limit the requested velocities...
        pandaArms.ArmL.xdot.pose(1:3) = Saturate(pandaArms.ArmL.xdot.pose(1:3), 0.7);
        pandaArms.ArmL.xdot.pose(4:6) = Saturate(pandaArms.ArmL.xdot.pose(4:6), 0.7);

        % RIGHT ARM
        % -----------------------------------------------------------------
        % Tool position and orientation task reference
        [ang, lin] = CartError(pandaArms.ArmR.wTg, pandaArms.ArmR.wTt);

        pandaArms.ArmR.xdot.pose = 0.2 * [ang; lin];
            % limit the requested velocities...
        pandaArms.ArmR.xdot.pose(1:3) = Saturate(pandaArms.ArmR.xdot.pose(1:3), 1);
        pandaArms.ArmR.xdot.pose(4:6) = Saturate(pandaArms.ArmR.xdot.pose(4:6), 1);

        jl_min = deg2rad([-166 -101 -166 -176 -166 -1 -166])';
        jl_max = deg2rad([166 101 166 -4 -166 115 166])';
        pandaArms.ArmL.xdot.jointLimits = zeros(7,1);
        pandaArms.ArmR.xdot.jointLimits = zeros(7,1);
        delta_perc = 0.1;

        %% Jl left
        index = 1;
        for jl = [jl_min; jl_max]
            if pandaArms.ArmL.q(index) > jl(2) - delta_perc * (jl(2) - jl(1))
                pandaArms.ArmL.xdot.jointLimits(index,1) = 0.2 * (pandaArms.ArmL.q(index) - (jl(2) - delta_perc * (jl(2) - jl(1))));
            end
            if pandaArms.ArmL.q(index) < jl(1) + delta_perc * (jl(2) - jl(1))
                pandaArms.ArmL.xdot.jointLimits(index,1) = 0.2 * ((jl(1) + delta_perc * (jl(2) - jl(1))) - pandaArms.ArmL.q(index));
            end

            if pandaArms.ArmR.q(index) > jl(2) - delta_perc * (jl(2) - jl(1))
                pandaArms.ArmR.xdot.jointLimits(index,1) = 0.2 * (pandaArms.ArmR.q(index) - (jl(2) - delta_perc * (jl(2) - jl(1))));
            end
            if pandaArms.ArmR.q(index) < jl(1) + delta_perc * (jl(2) - jl(1))
                pandaArms.ArmR.xdot.jointLimits(index,1) = 0.2 * ((jl(1) + delta_perc * (jl(2) - jl(1))) - pandaArms.ArmR.q(index));
            end
            index = index + 1;
        end

        pandaArms.ArmL.xdot.bimanualGrasp = zeros(7,1);
        pandaArms.ArmR.xdot.bimanualGrasp = zeros(7,1);
    % case 2
    %     % Perform the rigid grasp of the object and move it

    %     % COMMON
    %     % -----------------------------------------------------------------
    %     % Rigid Grasp Constraint
    %     pandaArms.xdot.rc = ...;

    %         % LEFT ARM
    %     % -----------------------------------------------------------------
    %     % Object position and orientation task reference
    %     [ang, lin] = CartError();
    %     pandaArms.ArmL.xdot.tool = ...;
    %         % limit the requested velocities...
    %     pandaArms.ArmL.xdot.tool(1:3) = Saturate();
    %     pandaArms.ArmL.xdot.tool(4:6) = Saturate();

    %     % RIGHT ARM
    %     % -----------------------------------------------------------------
    %     % Object position and orientation task reference
    %     [ang, lin] = CartError();
    %     pandaArms.ArmR.xdot.tool = ...;
    %         % limit the requested velocities...
    %     pandaArms.ArmR.xdot.tool(1:3) = Saturate();
    %     pandaArms.ArmR.xdot.tool(4:6) = Saturate();
    % case 3
    %     % Stop any motions
    %     % LEFT ARM
    %     % -----------------------------------------------------------------
    %     % Tool position and orientation task reference
    %     pandaArms.ArmL.xdot.tool(1:3) = ...;
    %         pandaArms.ArmL.xdot.tool(4:6) = ...;

    %         % RIGHT ARM
    %     % -----------------------------------------------------------------
    %     % Tool position and orientation task reference
    %     pandaArms.ArmR.xdot.tool(1:3) = ...;
    %         pandaArms.ArmR.xdot.tool(4:6) = ...;
            end
end
