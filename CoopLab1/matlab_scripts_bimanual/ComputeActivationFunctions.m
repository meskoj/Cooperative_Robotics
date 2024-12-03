function [pandaArms] = ComputeActivationFunctions(pandaArms, mission)
% Minimum Altitude Task ( > 0.15m, 0.05m delta )
pandaArms.ArmL.A.minimumAltitude = DecreasingBellShapedFunction(0.15, 0.2, 0, 1, pandaArms.ArmL.wTt(3,4));
pandaArms.ArmR.A.minimumAltitude = DecreasingBellShapedFunction(0.15, 0.2, 0, 1, pandaArms.ArmR.wTt(3,4));

%% Joint Limits Task
% Activation function: two combined sigmoids, which are at their maximum at the joint limits and approach zero between them
% Safety Task (inequality)
% delta is 10% of max error

%% LEFT ARM
% -----------------------------------------------------------------
index = 1;
for jl = [pandaArms.jlmin; pandaArms.jlmax]
    pandaArms.ArmL.A.jointLimits(index,index) = DecreasingBellShapedFunction(jl(1), jl(1) + pandaArms.delta_perc * (jl(2) - jl(1)), 0, 1, pandaArms.ArmL.q(index)) + IncreasingBellShapedFunction(jl(2) - pandaArms.delta_perc * (jl(2) - jl(1)), jl(2), 0, 1, pandaArms.ArmL.q(index));
    index = index + 1;
end

%% RIGHT ARM
% -----------------------------------------------------------------
index = 1;
for jl = [pandaArms.jlmin; pandaArms.jlmax]
    pandaArms.ArmR.A.jointLimits(index,index) = DecreasingBellShapedFunction(jl(1), jl(1) + pandaArms.delta_perc * (jl(2) - jl(1)), 0, 1, pandaArms.ArmR.q(index)) + IncreasingBellShapedFunction(jl(2) - pandaArms.delta_perc * (jl(2) - jl(1)), jl(2), 0, 1, pandaArms.ArmR.q(index));
    index = index + 1;
end

switch mission.phase
    case 1  % Reach the grasping point
        % Move-To
        pandaArms.ArmL.A.pose = eye(6);
        pandaArms.ArmR.A.pose = eye(6);

    case 2 % Move the object holding it firmly
        % Rigid Grasp Constraint
        pandaArms.ArmL.A.pose = zeros(6);
        pandaArms.ArmR.A.pose = zeros(6);

        pandaArms.ArmL.A.bimanualPose = eye(6);
        pandaArms.ArmR.A.bimanualPose = eye(6);

        pandaArms.ArmL.A.rigidConstraint = eye(6);
        pandaArms.ArmR.A.rigidConstraint = eye(6);

    case 3 % STOP any motion
        pandaArms.ArmL.A.bimanualPose = zeros(6);
        pandaArms.ArmR.A.bimanualPose = zeros(6);

        pandaArms.ArmL.A.rigidConstraint = zeros(6);
        pandaArms.ArmR.A.rigidConstraint = zeros(6);

        pandaArms.ArmL.A.stopMotors = eye(7);
        pandaArms.ArmR.A.stopMotors = eye(7);
end
end
