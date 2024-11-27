function [pandaArms] = ComputeActivationFunctions(pandaArms, mission)

% EQUALITY TASK ACTIVATION
switch mission.phase
    case 1  % Reach the grasping point
        % Move-To
        % pandaArms.A.tool = ...;
        pandaArms.ArmL.A.pose = eye(6);
        pandaArms.ArmR.A.pose = eye(6);

        pandaArms.ArmL.A.bimanualGrasp = zeros(6);
        pandaArms.ArmR.A.bimanualGrasp = zeros(6);
    case 2 % Move the object holding it firmly
        % Rigid Grasp Constraint

        pandaArms.ArmL.A.pose = zeros(6);
        pandaArms.ArmR.A.pose = zeros(6);

        pandaArms.ArmL.A.bimanualGrasp = eye(6);
        pandaArms.ArmR.A.bimanualGrasp = eye(6);
        % Move-To

    case 3 % STOP any motion

end
% INEQUALITY TASK ACTIVATION
% Minimum Altitude Task ( > 0.15m, 0.05m delta )
pandaArms.ArmL.A.minimumAltitude = DecreasingBellShapedFunction(0.15, 0.2, 0, 1, pandaArms.ArmL.wTt(3,4));
pandaArms.ArmR.A.minimumAltitude = DecreasingBellShapedFunction(0.15, 0.2, 0, 1, pandaArms.ArmR.wTt(3,4));

% Joint Limits Task
% Activation function: two combined sigmoids, which are at their maximum
% at the joint limits and approach zero between them
% Safety Task (inequality)
% delta is 10% of max error
% pandaArms.A.jl = ...; TODO
jl_min = deg2rad([-166 -101 -166 -176 -166 -1 -166]);
jl_max = deg2rad([166 101 166 -4 166 215 166]);
delta_perc = 0.1;

index = 1;
for jl = [jl_min; jl_max]
    pandaArms.ArmL.A.jointLimits(index,index) = DecreasingBellShapedFunction(jl(1), jl(1) + delta_perc * (jl(2) - jl(1)), 0, 1, pandaArms.ArmL.q(index)) + IncreasingBellShapedFunction(jl(2), jl(2) - delta_perc * (jl(2) - jl(1)), 0, 1, pandaArms.ArmL.q(index));
    index = index + 1;
end

index = 1;
for jl = [jl_min; jl_max]
    pandaArms.ArmR.A.jointLimits(index,index) = DecreasingBellShapedFunction(jl(1), jl(1) + delta_perc * (jl(2) - jl(1)), 0, 1, pandaArms.ArmR.q(index)) + IncreasingBellShapedFunction(jl(2), jl(2) - delta_perc * (jl(2) - jl(1)), 0, 1, pandaArms.ArmR.q(index));
    index = index + 1;
end
end
