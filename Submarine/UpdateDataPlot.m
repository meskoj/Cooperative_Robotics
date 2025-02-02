function [ plt ] = UpdateDataPlot( plt, uvms, t, loop )

% this function samples the variables contained in the structure uvms
% and saves them in arrays inside the struct plt
% this allows to have the time history of the data for later plots

% you can add whatever sampling you need to do additional plots
% plots are done in the PrintPlot.m script

plt.t(loop) = t;

plt.toolPos(:, loop) = uvms.wTt(1:3,4);

plt.q(:, loop) = uvms.q;
plt.q_dot(:, loop) = uvms.q_dot;

plt.p(:, loop) = uvms.p;
plt.p_dot(:, loop) = uvms.p_dot;

plt.toolx(:,loop) = uvms.wTt(1,4);
plt.tooly(:,loop) = uvms.wTt(2,4);

plt.activationFunctions(1, loop) = uvms.A.altitudeControlSafety;
plt.activationFunctions(2, loop) = uvms.A.altitudeControlAD;
plt.activationFunctions(3, loop) = uvms.A.horizontalAttitude(1);
plt.activationFunctions(4, loop) = uvms.A.vehiclePosition(1);
plt.activationFunctions(5, loop) = uvms.A.vehiclePositionXY(1);
plt.activationFunctions(6, loop) = uvms.A.headingControl(1);
plt.activationFunctions(7, loop) = uvms.A.armControl(1);
plt.activationFunctions(8, loop) = uvms.A.noMovement(1);

plt.transitionTimes = uvms.transitionTimes;

plt.misalignmentToNodule(:, loop) = uvms.theta_z;
plt.altitudeError(:, loop) = uvms.altitude;

[ang, lin] = CartError(uvms.wTt, uvms.wTg);
plt.toolToNodule(:, loop) = [ang;lin];
end
