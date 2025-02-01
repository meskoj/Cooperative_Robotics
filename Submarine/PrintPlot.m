function [ ] = PrintPlot( plt )

% some predefined plots
% you can add your own
saving = false;
baseFolder = "~/Documents/RobotEngPersonal/Y2S1/COOP/Franka/Submarine/Images/";

f1 = figure(1);
subplot(2,1,1);
hplot = plot(plt.t, plt.q);
set(hplot, 'LineWidth', 4);
l = legend('q_1','q_2','q_3','q_4','q_5','q_6','q_7');
l.Location = "northeastoutside";
title("Arm position");
xlabel("s")
ylabel("rad")
subplot(2,1,2);
hplot = plot(plt.t, plt.q_dot);
set(hplot, 'LineWidth', 4);
l = legend('qdot_1','qdot_2','qdot_3','qdot_4','qdot_5','qdot_6','qdot_7');
l.Location = "northeastoutside";
xlabel("s")
ylabel("rad/s")
title("Arm velocity");
f1.Position = [0,0,1920,1080];
if saving
    saveas(f1, baseFolder+"ArmPosVel.png")
end

f2 = figure(2);
subplot(2,1,1);
hplot = plot(plt.t, plt.p(1:3, :));
set(hplot, 'LineWidth', 4);
l = legend('x','y','z');
l.Location = "northeastoutside";
xlabel("s")
ylabel("m")
title("Body linear position");
subplot(2,1,2);
hplot = plot(plt.t, plt.p_dot(1:3, :));
set(hplot, 'LineWidth', 4);
l = legend('xdot', 'ydot','zdot');
l.Location = "northeastoutside";
xlabel("s")
ylabel("m/s")
title("Body linear velocity");
f2.Position = [0,0,1920,1080];
if saving
    saveas(f2, baseFolder+"BodyLinPosVel.png")
end

f3 = figure(3);
subplot(2,1,1);
hplot = plot(plt.t, plt.p(4:6, :));
set(hplot, 'LineWidth', 4);
xlabel("s")
ylabel("rad")
title("Body angular position");
setLabels(gca,plt)
l = legend('roll','pitch','yaw','Phase Transition');
l.Location = "northeastoutside";
subplot(2,1,2);
hplot = plot(plt.t, plt.p_dot(4:6, :));
set(hplot, 'LineWidth', 4);
xlabel("s")
ylabel("rad/s")
title("Body angular velocity");
setLabels(gca, plt)
l = legend('omega_x','omega_y','omega_z','Phase Transition');
l.Location = "northeastoutside";
f3.Position = [0,0,1920,1080];
if saving
    saveas(f3, baseFolder+"BodyAngPosVel.png")
end

f4 = figure(4);
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
l = legend("Altitude control safety", "Altitude control action defined", "Horizontal attitude", "Vehicle position", "Vehicle position xy", "Heading control", "Arm control", "No movement", "Phase transition");
l.Location = "northeastoutside";
title("Activation Functions")
f4.Position = [0,0,1920,1080];
if saving
    saveas(f4, baseFolder+"ActivationFunctions.png")
end

f5 = figure(5);
subplot(2,1,1);
hplot = plot(plt.t, plt.misalignmentToNodule);
set(hplot, 'LineWidth', 4);
title("Misalignment on the z axis")
xlabel("s")
ylabel("rad")
setLabels(gca, plt)
l = legend("Misalignment z", "Phase transition");
l.Location = "northeastoutside";
subplot(2,1,2);
hplot = plot(plt.t, plt.altitudeError);
set(hplot, 'LineWidth', 4);
xlabel("s")
ylabel("m")
title("Altitude error")
setLabels(gca, plt)
l = legend("Misalignment z", "Phase transition");
l.Location = "northeastoutside";
f5.Position = [0,0,1920,1080];
if saving
    saveas(f5, baseFolder+"Misalignment.png")
end

f6 = figure(6);
subplot(2,1,1);
hplot = plot(plt.t, plt.toolToNodule(4:6,:));
set(hplot, 'LineWidth', 4);
xlabel("s")
ylabel("m")
title("Linear distance to nodule")
setLabels(gca, plt)
yline(0)
l = legend('Linear distance to nodule X', 'Linear distance to nodule Y', 'Linear distance to nodule Z', 'Phase transition');
l.Location = "northeastoutside";
subplot(2,1,2);
hplot = plot(plt.t, plt.toolToNodule(1:3,:));
set(hplot, 'LineWidth', 4);
xlabel("s")
ylabel("rad")
title("Angular distance to nodule")
setLabels(gca, plt)
yline(0)
l = legend('Angular distance to nodule X', 'Angular distance to nodule Y', 'Angular distance to nodule Z', 'Phase transition');
l.Location = "northeastoutside";
f6.Position = [0,0,1920,1080];
if saving
    saveas(f6, baseFolder+"BodyDistAngLin.png")
end

f7 = figure(7);
hplot = plot(plt.t, plt.q_dot);
set(hplot, 'LineWidth', 4);
xlabel("s")
ylabel("rad/s")
title("Arm velocity");
setLabels(gca, plt)
l = legend('qdot_1','qdot_2','qdot_3','qdot_4','qdot_5','qdot_6','qdot_7',"Phase Transition");
l.Location = "northeastoutside";
f7.Position = [0,0,1920,1080];
if saving
    saveas(f7, baseFolder+"ArmVelocity.png")
end

f8 = figure(8);
hplot = plot(plt.t, plt.p_dot(1:3, :));
set(hplot, 'LineWidth', 4);
xlabel("s")
ylabel("m/s")
title("Body linear velocity");
setLabels(gca, plt)
l = legend('xdot', 'ydot','zdot', "Phase Transition");
l.Location = "northeastoutside";
f8.Position = [0,0,1920,1080];
if saving
    saveas(f8, baseFolder+"BodyLinearVelocity.png")
end
end

function setLabels(currentAxis, plt)
    xline(plt.transitionTimes, 'k--', 'LineWidth', 2, 'DisplayName', "Phase Transition");
    insert = @(a, x, n)cat(2,  x(1:n), a, x(n+1:end));
    auxArr = sort([0:5:plt.t(end), plt.transitionTimes]);
    strLabels=arrayfun(@(a)num2str(a),(0:5:plt.t(end)),'uni',0);
    strLabels = insert('Phase 1 End', strLabels, max(1, find(auxArr == plt.transitionTimes(1))-1));
    if length(plt.transitionTimes) == 2
        strLabels = insert('Phase 2 End', strLabels, max(1,find(auxArr == plt.transitionTimes(2))-1));
    end
    strLabels(find(auxArr == 20)) = {""};
    auxLabels = strLabels';
    set(currentAxis, 'xtick', auxArr, 'xticklabel', auxLabels)
    xtickangle(45)
    set(currentAxis,'fontsize',20)
end
