function run_B()
% RUN_B  Runs coursework B1-B4 using MATLAB.
% B1: design full information regulator (prints design steps & control law)
% B2: linear simulation -> plot y(t) and u(t) for x(0)=0
% B3: nonlinear simulation -> plot y(t) and u(t) for x(0)=0
% B4: repeat B1-B3 for omega = 1 and omega = 10 (also runs omega=0.1 baseline)

clc; close all;

% --- fixed parameters from the statement / your model
cfg.M = 1;
cfg.L = 1;
cfg.F = 1;
cfg.g = 9.81;

% --- B1 statement (and reused in B2-B4)
cfg.d1_amp    = 0.5;
cfg.d1_period = 50;
cfg.alpha     = 1;

% initial state x(0)=0 for B2/B3
cfg.x0 = zeros(4,1);

% exosystem initial condition for d2(t)=alpha*sin(omega t)
% use d2(0)=0, d2dot(0)=alpha*omega
cfg.d2_0 = 0;

% simulation horizon: long enough for omega=0.1 (period ~ 62.8s)
cfg.tEnd = 250;

% --- B4: repeat with omega = 1 and 10; include omega=0.1 baseline
omegaList = [0.1, 1, 10];

for w = omegaList
    cfg.omega   = w;
    cfg.d2dot_0 = cfg.alpha * cfg.omega;

    fprintf("\n=============================================\n");
    fprintf("Running B1-B3 with omega = %.3g\n", cfg.omega);
    fprintf("=============================================\n");

    % (B1) design
    [sys, sol, gains] = design_fullinfo_regulator(cfg);

    % (B2) linear simulation: plots y(t) and u(t)
    outL = simulate_linear_fullinfo(sys, gains, cfg);
    plot_y_u(outL, cfg, sprintf("B2 Linear (omega=%.3g)", cfg.omega));

    % (B3) nonlinear simulation (A1 model): plots y(t) and u(t)
    outNL = simulate_nonlinear_fullinfo(sys, gains, cfg);
    plot_y_u(outNL, cfg, sprintf("B3 Nonlinear (omega=%.3g)", cfg.omega));
end

end
