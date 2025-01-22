load("plots.mat")

figure(1);
subplot(2,1,1);
hplot = plot(plt.t, plt.q);
set(hplot, 'LineWidth', 1);
legend('q_1','q_2','q_3','q_4','q_5','q_6','q_7');
title("Arm position");
xlabel("s")
ylabel("rad")
subplot(2,1,2);
hplot = plot(plt.t, plt.q_dot);
set(hplot, 'LineWidth', 1);
legend('qdot_1','qdot_2','qdot_3','qdot_4','qdot_5','qdot_6','qdot_7');
xlabel("s")
ylabel("rad/s")
title("Arm velocity");

figure(2);
subplot(2,1,1);
hplot = plot(plt.t, plt.p(1:3, :));
set(hplot, 'LineWidth', 1);
legend('x','y','z');
xlabel("s")
ylabel("m")
title("Body linear position");
subplot(2,1,2);
hplot = plot(plt.t, plt.p_dot(1:3, :));
set(hplot, 'LineWidth', 1);
legend('xdot', 'ydot','zdot');
xlabel("s")
ylabel("m/s")
title("Body linear velocity");

figure(3);
subplot(2,1,1);
hplot = plot(plt.t, plt.p(4:6, :));
set(hplot, 'LineWidth', 1);
legend('roll','pitch','yaw');
xlabel("s")
ylabel("rad")
title("Body angular position");
subplot(2,1,2);
hplot = plot(plt.t, plt.p_dot(4:6, :));
set(hplot, 'LineWidth', 1);
xlabel("s")
ylabel("rad/s")
legend('omega_x','omega_y','omega_z');
title("Body angular velocity");

figure(4);
subplot(1,1,1);
% This was necessary because of the wrong initialization of the sensor distance
plt.activationFunctions(1,1:100) = zeros(1, 100);
hplot = plot(plt.t, plt.activationFunctions);
set(hplot, 'LineWidth', 4);
nameArray = {'LineStyle'};
valueArray = transpose({'-','-','--',':','--',':'});
colorArray = {'Color'};
colorNames = transpose({'#77AC30','#D95319','m','c','b','r'});
lineArray = {'LineWidth'};
lineVals = transpose({4,2.5,4,4,4,4});
set(hplot, nameArray, valueArray)
set(hplot, colorArray, colorNames)
set(hplot, lineArray, lineVals)
setLabels(gca, plt)
xlabel("s")
legend("Altitude control", "Horizontal attitude", "Vehicle position", "Heading control", "Arm control", "No movement", "Phase transition");
title("Activation Functions")

figure(5);
subplot(2,1,1);
hplot = plot(plt.t, plt.misalignmentToNodule);
set(hplot, 'LineWidth', 1);
legend('misalignment_z');
title("Misalignment on the z axis")
xlabel("s")
ylabel("rad")
subplot(2,1,2);
hplot = plot(plt.t, plt.altitudeError);
set(hplot, 'LineWidth', 1);
legend('Altitude error');
xlabel("s")
ylabel("m")
title("Altitude error")

figure(6)
subplot(1,1,1);
hplot = plot(plt.t, plt.distanceToNodule);
set(hplot, 'LineWidth', 1);
xlabel("s")
ylabel("m")
legend('distance to nodule X', 'distance to nodule Y', 'distance to nodule Z');
title("Distance to nodule")


function setLabels(currentAxis, plt)
    xline(plt.transitionTimes, 'k--', 'LineWidth', 2, 'DisplayName', "Phase Transition");
    insert = @(a, x, n)cat(2,  x(1:n), a, x(n+1:end));
    auxArr = sort([0:5:plt.t(end), plt.transitionTimes]);
    strLabels=arrayfun(@(a)num2str(a),(0:5:plt.t(end)),'uni',0);
    strLabels = insert('Phase 1', strLabels, max(1, find(auxArr == plt.transitionTimes(1))-1));
    strLabels = insert('Phase 2', strLabels, max(1,find(auxArr == plt.transitionTimes(2))-1));
    strLabels(find(auxArr == 20)) = {""};
    auxLabels = strLabels';
    set(currentAxis, 'xtick', auxArr, 'xticklabel', auxLabels)
    set(currentAxis,'fontsize',20)
    xtickangle(45)
end
