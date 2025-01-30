function [ ] = PrintPlot( plt)

% some predefined plots
% you can add your own

saving = true;
basePath = "/home/tonello/Documents/RobotEngPersonal/Y2S1/COOP/Franka/Cooperative_Manipulation/matlab_scripts_cooperation/Images/";

f1 = figure('Name', 'Joint position and velocity Left Arm');
subplot(2,1,1);
hplot = plot(plt.t, plt.q);
title('LEFT ARM');
set(hplot, 'LineWidth', 1);
setLabels(gca, plt);
legend('q_1','q_2','q_3','q_4','q_5','q_6','q_7',"PhaseTransition");
subplot(2,1,2);
hplot = plot(plt.t, plt.q_dot);
set(hplot, 'LineWidth', 1);
setLabels(gca, plt);
legend('qdot_1','qdot_2','qdot_3','qdot_4','qdot_5','qdot_6','qdot_7',"PhaseTransition");
if saving
    f1.Position = [0,0,1920,1080];
    saveas(f1, basePath+"JointPosVelL.png");
end

f2 = figure('Name', 'Joint position and velocity Right Arm');
subplot(2,1,1);
hplot = plot(plt.t, plt.q2);
title('RIGHT ARM');
set(hplot, 'LineWidth', 1);
setLabels(gca, plt);
legend('q_1','q_2','q_3','q_4','q_5','q_6','q_7', "PhaseTransition");
subplot(2,1,2);
hplot = plot(plt.t, plt.q_dot2);
set(hplot, 'LineWidth', 1);
setLabels(gca, plt);
legend('qdot_1','qdot_2','qdot_3','qdot_4','qdot_5','qdot_6','qdot_7', "PhaseTransition");
if saving
    f2.Position = [0,0,1920,1080];
    saveas(f1, basePath+"JointPosVelR.png");
end

f3 = figure('Name', 'ArmsLinError');
subplot(2,1,1);
hplot = plot(plt.t, plt.secondGoalErrorL);
title('l_arm');
set(hplot, 'LineWidth', 1);
setLabels(gca, plt);
legend('x','y','z', "PhaseTransition");
subplot(2,1,2);
hplot = plot(plt.t, plt.secondGoalErrorR);
set(hplot, 'LineWidth', 1);
setLabels(gca, plt);
legend('x','y','z', "PhaseTransition");
if saving
    f3.Position = [0,0,1920,1080];
    saveas(f1, basePath+"LinearErrorArmGoal.png");
end


f4 = figure('Name', 'RelativeDistance');
subplot(1,1,1);
hplot = plot(plt.t, plt.relativeDistance);
title('relativeDistance');
set(hplot, 'LineWidth', 1);
setLabels(gca, plt);
legend('x','y','z', "PhaseTransition");
if saving
    f4.Position = [0,0,1920,1080];
    saveas(f1, basePath+"RelDistance.png");
end
end
function setLabels(currentAxis, plt)
if length(plt.transitionTimes) == 2
    xline(plt.transitionTimes, 'k--', 'LineWidth', 2, 'DisplayName', "Phase Transition");
    insert = @(a, x, n)cat(2,  x(1:n), a, x(n+1:end));
    auxArr = sort([0:1:plt.t(end), plt.transitionTimes]);
    strLabels=arrayfun(@(a)num2str(a),(0:1:plt.t(end)),'uni',0);
    strLabels = insert('Phase 1', strLabels, max(1, find(auxArr == plt.transitionTimes(1))-1));
    strLabels = insert('Phase 2', strLabels, max(1,find(auxArr == plt.transitionTimes(2))-1));
    strLabels(find(auxArr == 16)) = {""};
    auxLabels = strLabels';
    set(currentAxis, 'xtick', auxArr, 'xticklabel', auxLabels)
    xtickangle(45)
end
set(currentAxis,'fontsize',20)
end
