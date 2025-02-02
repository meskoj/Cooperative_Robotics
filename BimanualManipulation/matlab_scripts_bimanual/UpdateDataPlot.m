function [ plt ] = UpdateDataPlot( plt, pandaArms, t, loop, mission )
    % this function samples the variables contained in the structure pandaArms
    % and saves them in arrays inside the struct plt
    % this allows to have the time history of the datauvms for later plots

    % you can add whatever sampling you need to do additional plots
    % plots are done in the PrintPlot.m script


    plt.t(:, loop) = t;
    plt.q(:, loop) = pandaArms.ArmL.q;
    plt.q_dot(:, loop) = pandaArms.ArmL.q_dot;
    plt.q2(:, loop) = pandaArms.ArmR.q;
    plt.q_dot2(:, loop) = pandaArms.ArmR.q_dot;
    plt.relativeDistance(:, loop) = pandaArms.ArmL.wTt(1:3,4) - pandaArms.ArmR.wTt(1:3,4);
    plt.toolError(:, loop) = pandaArms.ArmL.wTnt(1:3,4) - pandaArms.ArmR.wTnt(1:3,4);
    plt.v_objL(:,loop) = pandaArms.ArmL.xdot.bimanualPose;
    plt.v_objR(:,loop) = pandaArms.ArmL.xdot.bimanualPose;
    plt.xl(:,loop) = pandaArms.ArmL.x;
    plt.xr(:,loop) = pandaArms.ArmR.x;

    plt.leftJointLimitsActivation(:, loop) = diag(pandaArms.ArmL.A.jointLimits);
    plt.rightJointLimitsActivation(:, loop) = diag(pandaArms.ArmR.A.jointLimits);

    plt.leftActivationFunctions(1, loop) = pandaArms.ArmL.A.minimumAltitude;
    plt.leftActivationFunctions(2, loop) = pandaArms.ArmL.A.pose(1);
    plt.leftActivationFunctions(3, loop) = pandaArms.ArmL.A.bimanualPose(1);
    plt.leftActivationFunctions(4, loop) = pandaArms.ArmL.A.stopMotors(1);
    plt.leftActivationFunctions(5, loop) = pandaArms.ArmL.A.rigidConstraint(1);

    plt.rightActivationFunctions(1, loop) = pandaArms.ArmR.A.minimumAltitude;
    plt.rightActivationFunctions(2, loop) = pandaArms.ArmR.A.pose(1);
    plt.rightActivationFunctions(3, loop) = pandaArms.ArmR.A.bimanualPose(1);
    plt.rightActivationFunctions(4, loop) = pandaArms.ArmR.A.stopMotors(1);
    plt.rightActivationFunctions(5, loop) = pandaArms.ArmR.A.rigidConstraint(1);
end
