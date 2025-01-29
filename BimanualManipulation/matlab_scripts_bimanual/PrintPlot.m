function [ ] = PrintPlot( plt, pandaArms )

% some predefined plots
% you can add your own

saving = false;
basePath = "~/Documents/RobotEngPersonal/Y2S1/COOP/Franka/BimanualManipulation/matlab_scripts_bimanual/Images/";

f1 = figure('Name', 'Joint position and velocity Left Arm');
setLabels(gca, plt, pandaArms);
subplot(2,1,1);
hplot = plot(plt.t, plt.q);
title('LEFT ARM');
set(hplot, 'LineWidth', 1);
setLabels(gca, plt, pandaArms);
legend('q_1','q_2','q_3','q_4','q_5','q_6','q_7','PhaseTransition');
subplot(2,1,2);
hplot = plot(plt.t, plt.q_dot);
set(hplot, 'LineWidth', 1);
setLabels(gca, plt, pandaArms);
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
setLabels(gca, plt, pandaArms);
legend('q_1','q_2','q_3','q_4','q_5','q_6','q_7','PhaseTransition');
subplot(2,1,2);
size(plt.q_dot2)
hplot = plot(plt.t, plt.q_dot2);
set(hplot, 'LineWidth', 1);
setLabels(gca, plt, pandaArms);
legend('qdot_1','qdot_2','qdot_3','qdot_4','qdot_5','qdot_6','qdot_7','PhaseTransition');
if saving
    f2.Position = [0,0,1920,1080];
    saveas(f2, basePath+"JointPosVelR.png");
end

f3 = figure('Name', 'Relative Distance');
subplot(1,1,1);
hplot = plot(plt.t, plt.relativeDistance);
title('RelDistance');
set(hplot, 'LineWidth', 1);
setLabels(gca, plt, pandaArms);
legend('relativeDist.x','relativeDist.y','relativeDist.z','PhaseTransition');
if saving
    f3.Position = [0,0,1920,1080];
    saveas(f3, basePath+"RelativeDistance.png");
end

f4 = figure('Name', 'Tool Error');
subplot(1,1,1);
hplot = plot(plt.t, plt.toolError);
title('Tool Error');
set(hplot, 'LineWidth', 1);
setLabels(gca, plt, pandaArms);
legend('toolError.x','toolError.y','toolError.z','PhaseTransition');
if saving
    f4.Position = [0,0,1920,1080];
    saveas(f4, basePath+"ToolError.png");
end

f5 = figure('Name', 'Object Velocity');
% Here we could also put v_objR. They are the same
hplot = plot(plt.t, [plt.v_objL;plt.xl(4:6,:);plt.xr(4:6,:)]);
title('Object velocity');
set(hplot, 'LineWidth', 4);
nameArray = {'LineStyle'};
valueArray = transpose({'-','-','-','--','--','--',':',':',':'});
set(hplot, nameArray, valueArray)
setLabels(gca, plt, pandaArms);
legend('objectVel.x','objectVel.y','objectVel.z','leftVel.x','leftVel.y','leftVel.z','rightVel.x','rightVel.y','rightVel.z','PhaseTransition');
if saving
    f5.Position = [0,0,1920,1080];
    saveas(f5, basePath+"ObjAndToolVels.png");
end

f6 = figure('Name', "Object vel diff");
hplot = plot(plt.t, plt.v_objR - plt.v_objL);
title("Difference between left and right tool")
setLabels(gca, plt, pandaArms);
legend('velDiff')
if saving
    f6.Position = [0,0,1920,1080];
    saveas(f6, basePath+"ObjLRDiff.png");
end
end
function setLabels(currentAxis, plt, pandaArms)
if length(pandaArms.transitionTimes) == 2
    xline(pandaArms.transitionTimes, 'k--', 'LineWidth', 2, 'DisplayName', "Phase Transition");
    insert = @(a, x, n)cat(2,  x(1:n), a, x(n+1:end));
    auxArr = sort([0:1:plt.t(end), pandaArms.transitionTimes]);
    strLabels=arrayfun(@(a)num2str(a),(0:1:plt.t(end)),'uni',0);
    strLabels = insert('Phase 1', strLabels, max(1, find(auxArr == pandaArms.transitionTimes(1))-1));
    strLabels = insert('Phase 2', strLabels, max(1,find(auxArr == pandaArms.transitionTimes(2))-1));
    strLabels(find(auxArr == 4)) = {""};
    auxLabels = strLabels';
    set(currentAxis, 'xtick', auxArr, 'xticklabel', auxLabels)
    xtickangle(45)
end
set(currentAxis,'fontsize',20)
end

