function [pandaArms] = ComputeActivationFunctions(pandaArms, mission)
% Minimum Altitude Task ( > 0.15m, 0.05m delta )
pandaArms.ArmL.A.minimumAltitude = DecreasingBellShapedFunction(0.15, 0.2, 0, 1, pandaArms.ArmL.wTt(3,4)) *...
    ActionTransition("MA", mission.actions.(mission.prev_action), mission.actions.(mission.current_action), mission.phase_time, false);
pandaArms.ArmR.A.minimumAltitude = DecreasingBellShapedFunction(0.15, 0.2, 0, 1, pandaArms.ArmR.wTt(3,4)) *...
    ActionTransition("MA", mission.actions.(mission.prev_action), mission.actions.(mission.current_action), mission.phase_time, false);

%% Joint Limits Task
% Activation function: two combined sigmoids, which are at their maximum at the joint limits and approach zero between them
% Safety Task (inequality)
% delta is 10% of max error

%% LEFT ARM
% -----------------------------------------------------------------
index = 1;
for jl = [pandaArms.jlmin; pandaArms.jlmax]
    pandaArms.ArmL.A.jointLimits(index,index) = (DecreasingBellShapedFunction(jl(1), jl(1) + pandaArms.delta_perc * (jl(2) - jl(1)), 0, 1, pandaArms.ArmL.q(index)) + IncreasingBellShapedFunction(jl(2) - pandaArms.delta_perc * (jl(2) - jl(1)), jl(2), 0, 1, pandaArms.ArmL.q(index))) *...
        ActionTransition("JL", mission.actions.(mission.prev_action), mission.actions.(mission.current_action), mission.phase_time, false);
    index = index + 1;
end

%% RIGHT ARM
% -----------------------------------------------------------------
index = 1;
for jl = [pandaArms.jlmin; pandaArms.jlmax]
    pandaArms.ArmR.A.jointLimits(index,index) = (DecreasingBellShapedFunction(jl(1), jl(1) + pandaArms.delta_perc * (jl(2) - jl(1)), 0, 1, pandaArms.ArmR.q(index)) + IncreasingBellShapedFunction(jl(2) - pandaArms.delta_perc * (jl(2) - jl(1)), jl(2), 0, 1, pandaArms.ArmR.q(index)))*...
        ActionTransition("JL", mission.actions.(mission.prev_action), mission.actions.(mission.current_action), mission.phase_time, false);
    index = index + 1;
end

% Move-To
pandaArms.ArmL.A.pose = eye(6) * ActionTransition("T", mission.actions.(mission.prev_action), mission.actions.(mission.current_action), mission.phase_time, false);
pandaArms.ArmR.A.pose = eye(6) * ActionTransition("T", mission.actions.(mission.prev_action), mission.actions.(mission.current_action), mission.phase_time, false);

% Rigid Grasp Constraint

pandaArms.ArmL.A.bimanualPose = eye(6) * ActionTransition("BT", mission.actions.(mission.prev_action), mission.actions.(mission.current_action), mission.phase_time, false);
pandaArms.ArmR.A.bimanualPose = eye(6) * ActionTransition("BT", mission.actions.(mission.prev_action), mission.actions.(mission.current_action), mission.phase_time, false);

pandaArms.ArmL.A.rigidConstraint = eye(6)...
    * ActionTransition("BT", mission.actions.(mission.prev_action), mission.actions.(mission.current_action), mission.phase_time, true);
pandaArms.ArmR.A.rigidConstraint = eye(6)...
    * ActionTransition("BT", mission.actions.(mission.prev_action), mission.actions.(mission.current_action), mission.phase_time, true);

pandaArms.ArmL.A.stopMotors = eye(7) * ActionTransition("NM", mission.actions.(mission.prev_action), mission.actions.(mission.current_action), mission.phase_time, false);
pandaArms.ArmR.A.stopMotors = eye(7) * ActionTransition("NM", mission.actions.(mission.prev_action), mission.actions.(mission.current_action), mission.phase_time, false);
end
