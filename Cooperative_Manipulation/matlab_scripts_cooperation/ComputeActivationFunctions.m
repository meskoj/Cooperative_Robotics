function [pandaArm] = ComputeActivationFunctions(pandaArm,mission)

%% INEQUALITY TASK ACTIVATION
% Minimum Altitude Task ( > 0.15m, 0.05m delta )
pandaArm.A.minimumAltitude = DecreasingBellShapedFunction(0.15, 0.2, 0, 1, pandaArm.wTt(3, 4)) * ActionTransition("MA", mission.actions.(mission.prev_action).tasks, mission.actions.(mission.current_action).tasks, mission.phase_time);

% Joint Limits Task
% Activation function: two combined sigmoids, which are at their maximum
% at the joint limits and approach zero between them
% Safety Task (inequality)
% delta is 10% of max error
index = 1;
for jl = [pandaArm.jlmin; pandaArm.jlmax]
    pandaArm.A.jointLimits(index,index) = DecreasingBellShapedFunction(jl(1), jl(1) + pandaArm.delta_perc * (jl(2) - jl(1)), 0, 1, pandaArm.q(index)) + IncreasingBellShapedFunction(jl(2) - pandaArm.delta_perc * (jl(2) - jl(1)), jl(2), 0, 1, pandaArm.q(index));
    index = index + 1;
end
pandaArm.A.jointLimits = pandaArm.A.jointLimits * ActionTransition("JL", mission.actions.(mission.prev_action).tasks, mission.actions.(mission.current_action).tasks, mission.phase_time);

% EQUALITY TASK ACTIVATION
switch mission.phase
    case 1  % Reach the grasping point
        % Move-To
        pandaArm.A.moveTool = eye(6) * ActionTransition("T", mission.actions.(mission.prev_action).tasks, mission.actions.(mission.current_action).tasks, mission.phase_time);
    case 2 % Move the object holding it firmly
        % Rigid Grasp Constraint
        pandaArm.A.rigidConstraint = eye(6) * ActionTransition("RC", mission.actions.(mission.prev_action).tasks, mission.actions.(mission.current_action).tasks, mission.phase_time);

        % Move-To

    case 3 % STOP any motion

end

end
