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

end
