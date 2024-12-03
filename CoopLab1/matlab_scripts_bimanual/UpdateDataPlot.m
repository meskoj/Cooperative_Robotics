function [ plt ] = UpdateDataPlot( plt, pandaArms, t, loop, mission )

% this function samples the variables contained in the structure pandaArms
% and saves them in arrays inside the struct plt
% this allows to have the time history of the datauvms for later plots

% you can add whatever sampling you need to do additional plots
% plots are done in the PrintPlot.m script


plt.t(:, loop) = t;
plt.q(:, loop) = pandaArms.ArmL.q;
plt.q_dot(:, loop) = pandaArms.ArmL.q_dot;
plt.q2(:, loop) = pandaArms.ArmR.q;
plt.q_dot2(:, loop) = pandaArms.ArmR.q_dot;
plt.relativeDistance(:, loop) = pandaArms.ArmL.wTt(1:3,4) - pandaArms.ArmR.wTt(1:3,4);
plt.toolError(:, loop) = pandaArms.ArmL.wTnt(1:3,4) - pandaArms.ArmR.wTnt(1:3,4);

% activation functions


% Plot: desired object velocity

%End effector velocities (Left Arm)


%End effector velocities (Right Arm)


% Plot: manipulability task activation function


end
