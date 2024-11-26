function [pandaArms] = ComputeActivationFunctions(pandaArms, mission)

% EQUALITY TASK ACTIVATION
switch mission.phase
    case 1  % Reach the grasping point
        % Move-To
        % pandaArms.A.tool = ...;
    case 2 % Move the object holding it firmly
        % Rigid Grasp Constraint
        
        % Move-To
        
    case 3 % STOP any motion 
        
end
% INEQUALITY TASK ACTIVATION
% Minimum Altitude Task ( > 0.15m, 0.05m delta )
pandaArms.ArmL.A.minimumAltitude = DecreasingBellShapedFunction(0.15, 0.2, 0, 1, pandaArms.ArmL.wTt(3,4));
pandaArms.ArmR.A.minimumAltitude = DecreasingBellShapedFunction(0.15, 0.2, 0, 1, pandaArms.ArmR.wTt(3,4));

pandaArms.ArmL.A.pose = eye(6);
pandaArms.ArmR.A.pose = eye(6);

% Joint Limits Task
% Activation function: two combined sigmoids, which are at their maximum 
% at the joint limits and approach zero between them    
% Safety Task (inequality)
% delta is 10% of max error
% pandaArms.A.jl = ...; TODO
jl_min = deg2rad([-166 -101 -166 -176 -166 -1 -166]);
jl_max = deg2rad([166 101 166 -4 -166 115 166]);

index = 1;
for jl = [jl_min; jl_max]
    pandaArms.ArmL.A.jointLimits(index,index) = DecreasingBellShapedFunction(jl(1), jl(1) + 0.1 * (jl(2) - jl(1)), 0, 1, pandaArms.ArmL.q(index));
    if pandaArms.ArmL.A.jointLimits(index,index) == 0 % This in order to not have overwriting of the values
        pandaArms.ArmL.A.jointLimits(index,index) = IncreasingBellShapedFunction(jl(2), jl(2) - 0.1 * (jl(2) - jl(1)), 0, 1, pandaArms.ArmL.q(index));
    end
    index = index + 1;
end

index = 1;
for jl = [jl_min; jl_max]
    pandaArms.ArmR.A.jointLimits(index,index) = DecreasingBellShapedFunction(jl(1), jl(1) + 0.1 * (jl(2) - jl(1)), 0, 1, pandaArms.ArmR.q(index));
    if pandaArms.ArmR.A.jointLimits(index,index) == 0 % This in order to not have overwriting of the values
        pandaArms.ArmR.A.jointLimits(index,index) = IncreasingBellShapedFunction(jl(2), jl(2) - 0.1 * (jl(2) - jl(1)), 0, 1, pandaArms.ArmR.q(index));
    end
    index = index + 1;
end
