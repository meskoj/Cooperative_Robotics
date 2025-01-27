function [ ] = PrintPlot( plt )

% some predefined plots
% you can add your own
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
xlabel("s")
ylabel("rad")
title("Body angular position");
setLabels(gca,plt)
legend('roll','pitch','yaw','Phase Transition');
subplot(2,1,2);
hplot = plot(plt.t, plt.p_dot(4:6, :));
set(hplot, 'LineWidth', 1);
xlabel("s")
ylabel("rad/s")
title("Body angular velocity");
setLabels(gca, plt)
legend('omega_x','omega_y','omega_z','Phase Transition');

figure(4);
subplot(1,1,1);
% This was necessary because of the wrong initialization of the sensor distance
plt.activationFunctions(1,1:100) = zeros(1, 100);
hplot = plot(plt.t, plt.activationFunctions);
set(hplot, 'LineWidth', 4);
nameArray = {'LineStyle'};
valueArray = transpose({'-','-','-','--','--',':','--',':'});
colorArray = {'Color'};
colorNames = transpose({'#77AC30', '#EDB120','#D95319','m', '#7E2F8E','c','b','r'});
lineArray = {'LineWidth'};
lineVals = transpose({4,4,2.5,4,4,4,4,4});
set(hplot, nameArray, valueArray)
set(hplot, colorArray, colorNames)
set(hplot, lineArray, lineVals)
setLabels(gca, plt)
xlabel("s")
legend("Altitude control safety", "Altitude control action defined", "Horizontal attitude", "Vehicle position", "Vehicle position xy", "Heading control", "Arm control", "No movement", "Phase transition");
title("Activation Functions")

figure(5);
subplot(2,1,1);
hplot = plot(plt.t, plt.misalignmentToNodule);
set(hplot, 'LineWidth', 1);
title("Misalignment on the z axis")
xlabel("s")
ylabel("rad")
setLabels(gca, plt)
legend("Misalignment z", "Phase transition");
subplot(2,1,2);
hplot = plot(plt.t, plt.altitudeError);
set(hplot, 'LineWidth', 1);
xlabel("s")
ylabel("m")
title("Altitude error")
setLabels(gca, plt)
legend("Misalignment z", "Phase transition");

figure(6)
subplot(2,1,1);
hplot = plot(plt.t, plt.toolToNodule(4:6,:));
set(hplot, 'LineWidth', 1);
xlabel("s")
ylabel("m")
title("Linear distance to nodule")
setLabels(gca, plt)
yline(0)
legend('Linear distance to nodule X', 'Linear distance to nodule Y', 'Linear distance to nodule Z', 'Phase transition');
subplot(2,1,2);
hplot = plot(plt.t, plt.toolToNodule(1:3,:));
set(hplot, 'LineWidth', 1);
xlabel("s")
ylabel("rad")
title("Angular distance to nodule")
setLabels(gca, plt)
yline(0)
legend('Angular distance to nodule X', 'Angular distance to nodule Y', 'Angular distance to nodule Z', 'Phase transition');

figure(7)
hplot = plot(plt.t, plt.q_dot);
set(hplot, 'LineWidth', 1);
xlabel("s")
ylabel("rad/s")
title("Arm velocity");
setLabels(gca, plt)
legend('qdot_1','qdot_2','qdot_3','qdot_4','qdot_5','qdot_6','qdot_7',"Phase Transition");

figure(8)
hplot = plot(plt.t, plt.p_dot(1:3, :));
set(hplot, 'LineWidth', 1);
xlabel("s")
ylabel("m/s")
title("Body linear velocity");
setLabels(gca, plt)
legend('xdot', 'ydot','zdot', "Phase Transition");
end

function setLabels(currentAxis, plt)
    if length(plt.transitionTimes) == 2
        xline(plt.transitionTimes, 'k--', 'LineWidth', 2, 'DisplayName', "Phase Transition");
        insert = @(a, x, n)cat(2,  x(1:n), a, x(n+1:end));
        auxArr = sort([0:5:plt.t(end), plt.transitionTimes]);
        strLabels=arrayfun(@(a)num2str(a),(0:5:plt.t(end)),'uni',0);
        strLabels = insert('Phase 1', strLabels, max(1, find(auxArr == plt.transitionTimes(1))-1));
        strLabels = insert('Phase 2', strLabels, max(1,find(auxArr == plt.transitionTimes(2))-1));
        strLabels(find(auxArr == 20)) = {""};
        auxLabels = strLabels';
        set(currentAxis, 'xtick', auxArr, 'xticklabel', auxLabels)
        xtickangle(45)
    end
    set(currentAxis,'fontsize',20)
end
