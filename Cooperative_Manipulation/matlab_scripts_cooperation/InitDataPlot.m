function [plt] = InitDataPlot( maxloops)
    plt.t = zeros(1, maxloops);
    
    plt.q = zeros(7, maxloops);
    plt.q_dot = zeros(7, maxloops);
    plt.q2 = zeros(7, maxloops);
    plt.q_dot2 = zeros(7, maxloops);
    
    plt.secondGoalErrorL = zeros(3, maxloops);
    plt.secondGoalErrorR = zeros(3, maxloops);

    plt.relativeDistance = zeros(3, maxloops);

    plt.leftJointLimitsActivation = zeros(7, maxloops);
    plt.leftActivationFunctions = zeros(3, maxloops);
    plt.rightActivationFunctions = zeros(3, maxloops);

    plt.leftObjVel = zeros(6, maxloops);
    plt.rightObjVel = zeros(6, maxloops);
end

