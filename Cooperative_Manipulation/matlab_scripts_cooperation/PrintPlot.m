function [ ] = PrintPlot( plt)

% some predefined plots
% you can add your own

saving = false;
basePath = "/home/tonello/Documents/RobotEngPersonal/Y2S1/COOP/Franka/Cooperative_Manipulation/matlab_scripts_cooperation/Images/";

f1 = figure('Name', 'Joint position and velocity Left Arm');
subplot(2,1,1);
hplot = plot(plt.t, plt.q);
title('LEFT ARM');
set(hplot, 'LineWidth', 4);
setLabels(gca, plt);
legend('q_1','q_2','q_3','q_4','q_5','q_6','q_7',"PhaseTransition");
subplot(2,1,2);
hplot = plot(plt.t, plt.q_dot);
set(hplot, 'LineWidth', 4);
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
set(hplot, 'LineWidth', 4);
setLabels(gca, plt);
legend('q_1','q_2','q_3','q_4','q_5','q_6','q_7', "PhaseTransition");
subplot(2,1,2);
hplot = plot(plt.t, plt.q_dot2);
set(hplot, 'LineWidth', 4);
setLabels(gca, plt);
legend('qdot_1','qdot_2','qdot_3','qdot_4','qdot_5','qdot_6','qdot_7', "PhaseTransition");
if saving
    f2.Position = [0,0,1920,1080];
    saveas(f2, basePath+"JointPosVelR.png");
end

f3 = figure('Name', 'ArmsLinError');
subplot(2,1,1);
hplot = plot(plt.t, plt.secondGoalErrorL);
title('left arm');
set(hplot, 'LineWidth', 4);
setLabels(gca, plt);
legend('x','y','z', "PhaseTransition");
subplot(2,1,2);
hplot = plot(plt.t, plt.secondGoalErrorR);
title('right arm');
set(hplot, 'LineWidth', 4);
setLabels(gca, plt);
legend('x','y','z', "PhaseTransition");
if saving
    f3.Position = [0,0,1920,1080];
    saveas(f3, basePath+"LinearErrorArmGoal.png");
end


f4 = figure('Name', 'RelativeDistance');
subplot(1,1,1);
hplot = plot(plt.t, plt.relativeDistance);
title('Relative Distance');
set(hplot, 'LineWidth', 4);
setLabels(gca, plt);
legend('x','y','z', "PhaseTransition");
if saving
    f4.Position = [0,0,1920,1080];
    saveas(f4, basePath+"RelDistanceG2.png");
end

f5 = figure("Name", "Coop and not Coop Left");
subplot(2,1,1)
hplot = plot(plt.t, [plt.coopVelL(4:6,:); plt.nonCoopVelL(4:6,:)]);
title("Coop and not Coop Linear Vel Left");
set(hplot, 'LineWidth', 4);
setLabels(gca, plt);
legend("x_c","y_c","z_c","x_nc","y_nc","z_nc", "PhaseTransition");
subplot(2,1,2)
hplot = plot(plt.t, [plt.coopVelL(1:3,:); plt.nonCoopVelL(1:3,:)]);
setLabels(gca, plt);
set(hplot, 'LineWidth', 4);
title("Coop and not Coop Angular Vel Left");
setLabels(gca, plt);
legend("wx_c","wy_c","wz_c","wx_nc","wy_nc","wz_nc", "PhaseTransition");
if saving
    f5.Position = [0,0,1920,1080];
    saveas(f5, basePath+"CoopNotCoopLeftG2.png");
end
f6 = figure("Name", "Coop and not Coop Right");
subplot(2,1,1)
hplot = plot(plt.t, [plt.coopVelR(4:6,:); plt.nonCoopVelR(4:6,:)]);
title("Coop and not Coop Linear Vel Right");
set(hplot, 'LineWidth', 4);
setLabels(gca, plt);
legend("x_c","y_c","z_c","x_nc","y_nc","z_nc", "PhaseTransition");
subplot(2,1,2)
hplot = plot(plt.t, [plt.coopVelR(1:3,:); plt.nonCoopVelR(1:3,:)]);
set(hplot, 'LineWidth', 4);
title("Coop and not Coop Angular Vel Right");
setLabels(gca, plt);
legend("wx_c","wy_c","wz_c","wx_nc","wy_nc","wz_nc", "PhaseTransition");
if saving
    f6.Position = [0,0,1920,1080];
    saveas(f6, basePath+"CoopNotCoopRightG2.png");
end
end
function setLabels(currentAxis, plt)
    xline(plt.transitionTimes, 'k--', 'LineWidth', 2, 'DisplayName', "Phase Transition");
    insert = @(a, x, n)cat(2,  x(1:n), a, x(n+1:end));
    auxArr = sort([0:1:plt.t(end), plt.transitionTimes]);
    strLabels=arrayfun(@(a)num2str(a),(0:1:plt.t(end)),'uni',0);
    strLabels = insert('Phase 1', strLabels, max(1, find(auxArr == plt.transitionTimes(1))-1));
    if length(plt.transitionTimes) == 2
        strLabels = insert('Phase 2', strLabels, max(1,find(auxArr == plt.transitionTimes(2))-1));
    end
    strLabels(find(auxArr == 16)) = {""};
    auxLabels = strLabels';
    set(currentAxis, 'xtick', auxArr, 'xticklabel', auxLabels)
    xtickangle(45)
    set(currentAxis,'fontsize',20)
end
