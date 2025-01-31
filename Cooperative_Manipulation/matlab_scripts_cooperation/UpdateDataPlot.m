function [ plt ] = UpdateDataPlot( plt, pandaArm1, pandaArm2, t, loop, mission )


% this function samples the variables contained in the structure pandaArm
% and saves them in arrays inside the struct plt
% this allows to have the time history of the datauvms for later plots

% you can add whatever sampling you need to do additional plots
% plots are done in the PrintPlot.m script


plt.t(:, loop) = t;
plt.q(:, loop) = pandaArm1.q;
plt.q_dot(:, loop) = pandaArm1.q_dot;
plt.q2(:, loop) = pandaArm2.q;
plt.q_dot2(:, loop) = pandaArm2.q_dot;

% activation functions


% Plot: desired object velocity

%End effector velocities (Left Arm)


%End effector velocities (Right Arm)
if mission.phase ~= 2
    plt.secondGoalErrorL(:,loop) = zeros(3,1);
    plt.secondGoalErrorR(:,loop) = zeros(3,1);
else
    plt.secondGoalErrorL(:,loop) = pandaArm1.wTt(1:3, 4) - pandaArm1.wTog(1:3,4);
    plt.secondGoalErrorR(:,loop) = pandaArm2.wTt(1:3, 4) - pandaArm2.wTog(1:3,4);

end

plt.relativeDistance(:, loop) = pandaArm1.wTt(1:3, 4) - pandaArm2.wTt(1:3, 4);
plt.transitionTimes = mission.transitionTimes;

plt.coopVelL(:,loop) = pandaArm1.coopVel;
plt.nonCoopVelL(:,loop) = pandaArm1.nonCoopVel;

plt.coopVelR(:,loop) = pandaArm2.coopVel;
plt.nonCoopVelR(:,loop) = pandaArm2.nonCoopVel;

plt.leftJointLimitsActivation(:,loop) = diag(pandaArm1.A.jointLimits);
plt.rightJointLimitsActivation(:,loop) = diag(pandaArm2.A.jointLimits);

plt.leftActivationFunctions(1, loop) = pandaArm1.A.minimumAltitude;
plt.leftActivationFunctions(2, loop) = pandaArm1.A.moveTool(1);
plt.leftActivationFunctions(3, loop) = pandaArm1.A.moveToolWithConstraint(1);
plt.leftActivationFunctions(4, loop) = pandaArm1.A.stopAll(1);

plt.rightActivationFunctions(1, loop) = pandaArm2.A.minimumAltitude;
plt.rightActivationFunctions(2, loop) = pandaArm2.A.moveTool(1);
plt.rightActivationFunctions(3, loop) = pandaArm2.A.moveToolWithConstraint(1);
plt.rightActivationFunctions(4, loop) = pandaArm2.A.stopAll(1);
end
