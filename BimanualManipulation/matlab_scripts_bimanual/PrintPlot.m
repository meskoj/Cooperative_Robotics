function [ ] = PrintPlot( plt, pandaArms )

% some predefined plots
% you can add your own

saving = false;
basePath = "~/Documents/RobotEngPersonal/Y2S1/COOP/Franka/BimanualManipulation/matlab_scripts_bimanual/Images/";

f1 = figure('Name', 'Joint position and velocity Left Arm');
setLabels(gca, plt, pandaArms);
subplot(2,1,1);
hplot = plot(plt.t, plt.q);
xlabel("s");
ylabel("rad");
title('Joint position and velocity Left Arm');
set(hplot, 'LineWidth', 4);
setLabels(gca, plt, pandaArms);
l = legend('q_1','q_2','q_3','q_4','q_5','q_6','q_7','PhaseTransition');
l.Location = "northeastoutside";
subplot(2,1,2);
hplot = plot(plt.t, plt.q_dot);
xlabel("s");
ylabel("rad/s");
set(hplot, 'LineWidth', 4);
setLabels(gca, plt, pandaArms);
l = legend('qdot_1','qdot_2','qdot_3','qdot_4','qdot_5','qdot_6','qdot_7',"PhaseTransition");
l.Location = "northeastoutside";
if saving
    f1.Position = [0,0,1920,1080];
    saveas(f1, basePath+"JointPosVelL.png");
end

f2 = figure('Name', 'Joint position and velocity Right Arm');
subplot(2,1,1);
hplot = plot(plt.t, plt.q2);
xlabel("s");
ylabel("rad");
title('Joint position and velocity Right Arm');
set(hplot, 'LineWidth', 4);
setLabels(gca, plt, pandaArms);
l = legend('q_1','q_2','q_3','q_4','q_5','q_6','q_7','PhaseTransition');
l.Location = "northeastoutside";
subplot(2,1,2);
size(plt.q_dot2)
hplot = plot(plt.t, plt.q_dot2);
xlabel("s");
ylabel("rad/s");
set(hplot, 'LineWidth', 4);
setLabels(gca, plt, pandaArms);
l = legend('qdot_1','qdot_2','qdot_3','qdot_4','qdot_5','qdot_6','qdot_7','PhaseTransition');
l.Location = "northeastoutside";
if saving
    f2.Position = [0,0,1920,1080];
    saveas(f2, basePath+"JointPosVelR.png");
end

f3 = figure('Name', 'Relative Distance');
subplot(1,1,1);
hplot = plot(plt.t, plt.relativeDistance);
xlabel("s");
ylabel("m");
title('RelDistance');
set(hplot, 'LineWidth', 4);
setLabels(gca, plt, pandaArms);
l = legend('relativeDist.x','relativeDist.y','relativeDist.z','PhaseTransition');
l.Location = "northeastoutside";
if saving
    f3.Position = [0,0,1920,1080];
    saveas(f3, basePath+"RelativeDistance.png");
end

f4 = figure('Name', 'Tool Error');
subplot(1,1,1);
hplot = plot(plt.t, plt.toolError);
xlabel("s");
ylabel("m");
title('Tool Error');
set(hplot, 'LineWidth', 4);
setLabels(gca, plt, pandaArms);
l = legend('toolError.x','toolError.y','toolError.z','PhaseTransition');
l.Location = "northeastoutside";
if saving
    f4.Position = [0,0,1920,1080];
    saveas(f4, basePath+"ToolError.png");
end

f5 = figure('Name', 'Object Velocity');
% Here we could also put v_objR. They are the same
subplot(2,1,1);
hplot = plot(plt.t, [plt.v_objL(4:6,:); plt.xl(4:6,:)]);
title('Left Linear vel object and tool');
set(hplot, 'LineWidth', 4);
xlabel("s")
ylabel("m/s")
setLabels(gca, plt, pandaArms);
l = legend('objectLinVel.x','objectLinVel.y','objectLinVel.z','leftLinVel.x','leftLinVel.y','leftLinVel.z','PhaseTransition');
l.Location = "northeastoutside";
subplot(2,1,2)
hplot = plot(plt.t, [plt.v_objL(1:3, :);plt.xl(1:3,:)]);
xlabel("s")
ylabel("rad/s")
title('Left Angular vel object and tool');
set(hplot, 'LineWidth', 4);
setLabels(gca, plt, pandaArms);
l = legend('objectAngVel.x','objectAngVel.y','objectAngVel.z','leftAngVel.x','leftAngVel.y','leftAngVel.z','PhaseTransition');
l.Location = "northeastoutside";
if saving
    f5.Position = [0,0,1920,1080];
    saveas(f5, basePath+"LeftObjAndToolVels.png");
end

f6 = figure('Name', 'Object Velocity');
% Here we could also put v_objR. They are the same
subplot(2,1,1);
hplot = plot(plt.t, [plt.v_objR(4:6,:); plt.xr(4:6,:)]);
xlabel("s")
ylabel("m/s")
title('Right Linear vel object and tool');
set(hplot, 'LineWidth', 4);
% nameArray = {'LineStyle'};
% valueArray = transpose({'-','-','-','--','--','--',':',':',':'});
% set(hplot, nameArray, valueArray)
setLabels(gca, plt, pandaArms);
l = legend('objectLinVel.x','objectLinVel.y','objectLinVel.z','rightLinVel.x','rightLinVel.y','rightLinVel.z','PhaseTransition');
l.Location = "northeastoutside";
subplot(2,1,2)
hplot = plot(plt.t, [plt.v_objR(1:3, :);plt.xr(1:3,:)]);
title('Right Angular vel object and tool');
set(hplot, 'LineWidth', 4);
xlabel("s")
ylabel("rad/s")
% nameArray = {'LineStyle'};
% valueArray = transpose({'-','-','-','--','--','--',':',':',':'});
% set(hplot, nameArray, valueArray)
setLabels(gca, plt, pandaArms);
l = legend('objectAngVel.x','objectAngVel.y','objectAngVel.z','rightAngVel.x','rightAngVel.y','rightAngVel.z','PhaseTransition');
l.Location = "northeastoutside";
if saving
    f6.Position = [0,0,1920,1080];
    saveas(f6, basePath+"RightObjAndToolVels.png");
end

f7 = figure('Name', "Object vel diff");
hplot = plot(plt.t, plt.v_objR - plt.v_objL);
title("Linear velocity difference between left and right tool")
set(hplot, 'LineWidth', 7);
setLabels(gca, plt, pandaArms);
xlabel("s")
ylabel("m/s")
l = legend('velDiff');
l.Location = "northeastoutside";
if saving
    f7.Position = [0,0,1920,1080];
    saveas(f7, basePath+"ObjLRDiff.png");
end

f8 = figure(8);
% This was necessary because of the wrong initialization of the sensor distance
subplot(2,1,1);
hplot = plot(plt.t, plt.leftJointLimitsActivation);
set(hplot, 'LineWidth', 4);
setLabels(gca, plt, pandaArms)
xlabel("s")
l = legend("JL1", "JL2", "JL3", "JL4", "JL5", "JL6", "JL7", "Phase transition");
l.Location = "northeastoutside";
title("Left joint limits activation functions")
subplot(2,1,2);
hplot = plot(plt.t, plt.rightJointLimitsActivation);
set(hplot, 'LineWidth', 4);
setLabels(gca, plt, pandaArms)
xlabel("s")
l = legend("JL1", "JL2", "JL3", "JL4", "JL5", "JL6", "JL7", "Phase transition");
l.Location = "northeastoutside";
title("Right joint limits activation functions")
if saving
    f8.Position = [0,0,1920,1080];
    saveas(f8, basePath+"JLActivationFunctions.png")
end


f9 = figure(9);
% This was necessary because of the wrong initialization of the sensor distance
subplot(2,1,1);
hplot = plot(plt.t, plt.leftActivationFunctions);
set(hplot, 'LineWidth', 4);
setLabels(gca, plt, pandaArms)
xlabel("s")
l = legend("Minimum Altitude", "Move Tool", "Move Tool Bimanual", "Stop All", "Rigid Constraint", "Phase transition");
l.Location = "northeastoutside";
nameArray = {'LineStyle'};
valueArray = transpose({'-','--',':','--',':'});
set(hplot, nameArray, valueArray)
title("Left Arm Activation Functions")
subplot(2,1,2);
hplot = plot(plt.t, plt.rightActivationFunctions);
set(hplot, 'LineWidth', 4);
setLabels(gca, plt, pandaArms)
xlabel("s")
l = legend("Minimum Altitude", "Move Tool", "Move Tool Bimanual", "Stop All", "Rigid constraint", "Phase transition");
l.Location = "northeastoutside";
nameArray = {'LineStyle'};
valueArray = transpose({'-','--',':','--', ':'});
set(hplot, nameArray, valueArray)
title("Right Arm Activation Functions")
if saving
    f9.Position = [0,0,1920,1080];
    saveas(f9, basePath+"ActivationFunctions.png")
end
end
function setLabels(currentAxis, plt, pandaArms)
xline(pandaArms.transitionTimes, 'k--', 'LineWidth', 2, 'DisplayName', "Phase Transition");
insert = @(a, x, n)cat(2,  x(1:n), a, x(n+1:end));
auxArr = sort([0:1:plt.t(end), pandaArms.transitionTimes]);
strLabels=arrayfun(@(a)num2str(a),(0:1:plt.t(end)),'uni',0);
strLabels = insert('Phase 1 End', strLabels, max(1, find(auxArr == pandaArms.transitionTimes(1))-1));
if length(pandaArms.transitionTimes) == 2
    strLabels = insert('Phase 2 End', strLabels, max(1,find(auxArr == pandaArms.transitionTimes(2))-1));
end
strLabels(find(auxArr == 4)) = {""};
auxLabels = strLabels';
set(currentAxis, 'xtick', auxArr, 'xticklabel', auxLabels)
xtickangle(45)
set(currentAxis,'fontsize',20)
end

