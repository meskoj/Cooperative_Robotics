function [plt] = InitDataPlot( maxloops)
    plt.t = zeros(1, maxloops);

    plt.q = zeros(7, maxloops);
    plt.q_dot = zeros(7, maxloops);
    plt.q2 = zeros(7, maxloops);
    plt.q_dot2 = zeros(7, maxloops);
    plt.relativeDistance = zeros(3, maxloops);
    plt.toolError = zeros(3, maxloops);
    plt.v_objL = zeros(6, maxloops);
    plt.v_objR = zeros(6, maxloops);
    plt.leftJointLimitsActivation = zeros(7, maxloops);
    plt.rightJointLimitsActivation = zeros(7, maxloops);
    plt.leftActivationFunctions = zeros(5, maxloops);
    plt.rightActivationFunctions = zeros(5, maxloops);
end

