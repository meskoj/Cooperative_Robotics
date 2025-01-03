function [ ] = PrintPlot( plt )

% some predefined plots
% you can add your own

figure(1);
subplot(2,1,1);
hplot = plot(plt.t, plt.q);
set(hplot, 'LineWidth', 1);
legend('q_1','q_2','q_3','q_4','q_5','q_6','q_7');
subplot(2,1,2);
hplot = plot(plt.t, plt.q_dot);
set(hplot, 'LineWidth', 1);
legend('qdot_1','qdot_2','qdot_3','qdot_4','qdot_5','qdot_6','qdot_7');
title("Arm position and velocity");

figure(2);
subplot(3,1,1);
hplot = plot(plt.t, plt.p);
set(hplot, 'LineWidth', 1);
legend('x','y','z','roll','pitch','yaw');
subplot(2,1,2);
hplot = plot(plt.t, plt.p_dot);
set(hplot, 'LineWidth', 1);
legend('xdot', 'ydot','zdot','omega_x','omega_y','omega_z');
title("Body position and velocity");

figure(3);
subplot(1,1,1);
hplot = plot(plt.t, plt.activationFunctions);
xline(plt.transitionTimes, 'b--', 'LineWidth', 3);
set(hplot, 'LineWidth', 1);
legend("altitudeControl", "horizontalAttitude", "vehiclePosition", "headingControl", "armControl", "noMovement");
title("Activation Functions")

figure(4);
subplot(2,1,1);
hplot = plot(plt.t, plt.misalignmentToNodule);
set(hplot, 'LineWidth', 1);
legend('misalignment_z');
subplot(2,1,2);
hplot = plot(plt.t, plt.altitudeError);
set(hplot, 'LineWidth', 1);
legend('Altitude error');
title("Q4")

figure(5)
subplot(1,1,1);
hplot = plot(plt.t, plt.distanceToNodule);
set(hplot, 'LineWidth', 1);
legend('distance to nodule X', 'distance to nodule Y', 'distance to nodule Z');
title("Q5")
end

