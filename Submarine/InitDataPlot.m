function [plt] = InitDataPlot( maxloops)
    plt.t = zeros(1, maxloops);
    plt.q = zeros(7, maxloops);
    plt.q_dot = zeros(7, maxloops);

    plt.p = zeros(6, maxloops);
    plt.p_dot = zeros(6, maxloops);

    plt.xdot_jl = zeros(7, maxloops);
    plt.xdot_mu = zeros(1, maxloops);
    plt.xdot_t = zeros(6, maxloops);

    plt.a = zeros(11, maxloops);

    plt.activationFunctions = zeros(6, maxloops);

    plt.altitudeError = zeros(1, maxloops);
    plt.misalignmentToNodule = zeros(1, maxloops);

    plt.toolToNodule = zeros(6, maxloops);
end

