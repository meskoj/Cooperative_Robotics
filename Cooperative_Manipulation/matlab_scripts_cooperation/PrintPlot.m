function [ ] = PrintPlot( plt)

% some predefined plots
% you can add your own

saving = true;
basePath = "/home/tonello/Documents/RobotEngPersonal/Y2S1/COOP/Franka/Cooperative_Manipulation/matlab_scripts_cooperation/Images/";

f1 = figure('Name', 'Joint position and velocity Left Arm');
subplot(2,1,1);
hplot = plot(plt.t, plt.q);
title('LEFT ARM');
set(hplot, 'LineWidth', 4);
setLabels(gca, plt);
xlabel("s")
ylabel("rad")
l = legend('q_1','q_2','q_3','q_4','q_5','q_6','q_7',"PhaseTransition");
l.Location = "northeastoutside";
subplot(2,1,2);
hplot = plot(plt.t, plt.q_dot);
xlabel("s")
ylabel("rad/s")
set(hplot, 'LineWidth', 4);
setLabels(gca, plt);
l = legend('qdot_1','qdot_2','qdot_3','qdot_4','qdot_5','qdot_6','qdot_7',"PhaseTransition");
l.Location = "northeastoutside";
if saving
    f1.Position = [0,0,1920,1080];
    saveas(f1, basePath+"JointPosVelL.png");
end

f2 = figure('Name', 'Joint position and velocity Right Arm');
subplot(2,1,1);
hplot = plot(plt.t, plt.q2);
title('RIGHT ARM');
set(hplot, 'LineWidth', 4);
xlabel("s")
xlabel("rad")
setLabels(gca, plt);
l = legend('q_1','q_2','q_3','q_4','q_5','q_6','q_7', "PhaseTransition");
l.Location = "northeastoutside";
subplot(2,1,2);
hplot = plot(plt.t, plt.q_dot2);
set(hplot, 'LineWidth', 4);
xlabel("s")
ylabel("rad/s")
setLabels(gca, plt);
l = legend('qdot_1','qdot_2','qdot_3','qdot_4','qdot_5','qdot_6','qdot_7', "PhaseTransition");
l.Location = "northeastoutside";
if saving
    f2.Position = [0,0,1920,1080];
    saveas(f2, basePath+"JointPosVelR.png");
end

f3 = figure('Name', 'ArmsLinError');
subplot(2,1,1);
hplot = plot(plt.t, plt.secondGoalErrorL);
title('Linear error to goal left arm');
set(hplot, 'LineWidth', 4);
setLabels(gca, plt);
l = legend('x','y','z', "PhaseTransition");
l.Location = "northeastoutside";
xlabel("s")
ylabel("m")
subplot(2,1,2);
hplot = plot(plt.t, plt.secondGoalErrorR);
xlabel("s")
ylabel("m")
title('Linear error to goal right arm');
set(hplot, 'LineWidth', 4);
setLabels(gca, plt);
l = legend('x','y','z', "PhaseTransition");
l.Location = "northeastoutside";
if saving
    f3.Position = [0,0,1920,1080];
    saveas(f3, basePath+"LinearErrorArmGoal.png");
end


f4 = figure('Name', 'RelativeDistance');
subplot(1,1,1);
hplot = plot(plt.t, plt.relativeDistance);
title('Relative Distance');
xlabel("s");
ylabel("m");
set(hplot, 'LineWidth', 4);
setLabels(gca, plt);
l = legend('x','y','z', "PhaseTransition");
l.Location = "northeastoutside";
if saving
    f4.Position = [0,0,1920,1080];
    saveas(f4, basePath+"RelDistance.png");
end

f5 = figure("Name", "Coop and not Coop Left");
subplot(2,1,1)
hplot = plot(plt.t, [plt.coopVelL(4:6,:); plt.nonCoopVelL(4:6,:)]);
title("Coop and not Coop Linear Vel Left");
set(hplot, 'LineWidth', 4);
setLabels(gca, plt);
xlabel("s");
ylabel("m/s");
l = legend("x_c","y_c","z_c","x_{nc}","y_{nc}","z_{nc}", "PhaseTransition");
l.Location = "northeastoutside";
nameArray = {'LineStyle'};
valueArray = transpose({'--','--','--',':',':',':'});
set(hplot, nameArray, valueArray)
subplot(2,1,2)
hplot = plot(plt.t, [plt.coopVelL(1:3,:); plt.nonCoopVelL(1:3,:)]);
setLabels(gca, plt);
set(hplot, 'LineWidth', 4);
title("Coop and not Coop Angular Vel Left");
nameArray = {'LineStyle'};
valueArray = transpose({'--','--','--',':',':',':'});
set(hplot, nameArray, valueArray)
xlabel("s");
ylabel("rad/s");
setLabels(gca, plt);
l = legend("wx_c","wy_c","wz_c","wx_{nc}","wy_{nc}","wz_{nc}", "PhaseTransition");
l.Location = "northeastoutside";
if saving
    f5.Position = [0,0,1920,1080];
    saveas(f5, basePath+"CoopNotCoopLeft.png");
end

f6 = figure("Name", "Coop and not Coop Right");
subplot(2,1,1)
hplot = plot(plt.t, [plt.coopVelR(4:6,:); plt.nonCoopVelR(4:6,:)]);
title("Coop and not Coop Linear Vel Right");
set(hplot, 'LineWidth', 4);
xlabel("s");
ylabel("m/s");
setLabels(gca, plt);
l = legend("x_c","y_c","z_c","x_{nc}","y_{nc}","z_{nc}", "PhaseTransition");
l.Location = "northeastoutside";
nameArray = {'LineStyle'};
valueArray = transpose({'--','--','--',':',':',':'});
set(hplot, nameArray, valueArray)
subplot(2,1,2)
hplot = plot(plt.t, [plt.coopVelR(1:3,:); plt.nonCoopVelR(1:3,:)]);
set(hplot, 'LineWidth', 4);
title("Coop and not Coop Angular Vel Right");
setLabels(gca, plt);
xlabel("s");
ylabel("rad/s");
l = legend("wx_c","wy_c","wz_c","wx_{nc}","wy_{nc}","wz_{nc}", "PhaseTransition");
l.Location = "northeastoutside";
nameArray = {'LineStyle'};
valueArray = transpose({'--','--','--',':',':',':'});
set(hplot, nameArray, valueArray)
if saving
    f6.Position = [0,0,1920,1080];
    saveas(f6, basePath+"CoopNotCoopRight.png");
end

f7 = figure(7);
% This was necessary because of the wrong initialization of the sensor distance
subplot(2,1,1);
hplot = plot(plt.t, plt.leftJointLimitsActivation);
set(hplot, 'LineWidth', 4);
setLabels(gca, plt)
xlabel("s")
l = legend("JL1", "JL2", "JL3", "JL4", "JL5", "JL6", "JL7", "Phase transition");
l.Location = "northeastoutside";
title("Left joint limits activation functions")
subplot(2,1,2);
hplot = plot(plt.t, plt.rightJointLimitsActivation);
set(hplot, 'LineWidth', 4);
setLabels(gca, plt)
xlabel("s")
l = legend("JL1", "JL2", "JL3", "JL4", "JL5", "JL6", "JL7", "Phase transition");
l.Location = "northeastoutside";
title("Right joint limits activation functions")
if saving
    f7.Position = [0,0,1920,1080];
    saveas(f7, basePath+"LeftJLActivationFunctions.png")
end


f8 = figure(8);
% This was necessary because of the wrong initialization of the sensor distance
subplot(2,1,1);
hplot = plot(plt.t, plt.leftActivationFunctions);
set(hplot, 'LineWidth', 4);
setLabels(gca, plt)
xlabel("s")
l = legend("Minimum Altitude", "Move Tool", "Stop All", "Phase transition");
l.Location = "northeastoutside";
nameArray = {'LineStyle'};
valueArray = transpose({'-','--',':'});
set(hplot, nameArray, valueArray)
title("Left Robot Activation Functions")
subplot(2,1,2);
hplot = plot(plt.t, plt.rightActivationFunctions);
set(hplot, 'LineWidth', 4);
setLabels(gca, plt)
xlabel("s")
l = legend("Minimum Altitude", "Move Tool", "Stop All", "Phase transition");
l.Location = "northeastoutside";
nameArray = {'LineStyle'};
valueArray = transpose({'-','--',':'});
set(hplot, nameArray, valueArray)
title("Right Robot Activation Functions")
if saving
    f8.Position = [0,0,1920,1080];
    saveas(f8, basePath+"ActivationFunctions.png")
end
end

function setLabels(currentAxis, plt)
xline(plt.transitionTimes, 'k--', 'LineWidth', 2, 'DisplayName', "Phase Transition");
insert = @(a, x, n)cat(2,  x(1:n), a, x(n+1:end));
auxArr = sort([0:1:plt.t(end), plt.transitionTimes]);
strLabels=arrayfun(@(a)num2str(a),(0:1:plt.t(end)),'uni',0);
strLabels = insert('Phase 1 end', strLabels, max(1, find(auxArr == plt.transitionTimes(1))-1));
if length(plt.transitionTimes) == 2
    strLabels = insert('Phase 2 end', strLabels, max(1,find(auxArr == plt.transitionTimes(2))-1));
end
strLabels(find(auxArr == 9)) = {""};
auxLabels = strLabels';
set(currentAxis, 'xtick', auxArr, 'xticklabel', auxLabels)
xtickangle(45)
set(currentAxis,'fontsize',20)
end
