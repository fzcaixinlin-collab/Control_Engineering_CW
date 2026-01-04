function run_errors_01_1()
% RUN_ERRORS_01_1
% Generate ONLY the error plots for omega = 0.1 and omega = 1:
%   - B2 Linear:    e_s(t) = s(t) - d2(t),  e_phi(t) = phi(t)
%   - B3 Nonlinear: e_s(t) = s(t) - d2(t),  e_phi(t) = phi(t)
% Total: 8 figures.

clc; close all;

% -------------------- fixed parameters (match your coursework) --------------------
cfg.M = 1;
cfg.L = 1;
cfg.F = 1;
cfg.g = 9.81;

cfg.alpha     = 1;
cfg.d1_amp    = 0.5;
cfg.d1_period = 50;

cfg.x0 = zeros(4,1);  % x(0)=0

% For d2(t)=alpha*sin(omega t): encode amplitude via initial condition
% Choose d2(0)=0, d2dot(0)=alpha*omega
cfg.d2_0 = 0;

% Simulation horizon (safe for omega<=1)
cfg.tEnd = 250;

% Fixed output grid to avoid memory growth
dt = 5e-3;                 % 0.005 s step -> 50,001 points over 250s
tspan = 0:dt:cfg.tEnd;

% -------------------- omegas to run --------------------
omega_list = [0.1, 1];

for omega = omega_list

    cfg.omega = omega;
    cfg.d2dot_0 = cfg.alpha * cfg.omega;

    % Design regulator (B1 ingredients, for this omega)
    [sys, ~, gains] = design_fullinfo_regulator(cfg);

    % -------------------- B2 Linear errors --------------------
    outLin = simulate_linear_fullinfo_fixedgrid(sys, gains, cfg, tspan);
    plot_error_signals(outLin, cfg, sprintf("B2 Linear (omega=%.2g)", omega));

    % -------------------- B3 Nonlinear errors --------------------
    outNL = simulate_nonlinear_fullinfo_fixedgrid(sys, gains, cfg, tspan);
    plot_error_signals(outNL, cfg, sprintf("B3 Nonlinear (omega=%.2g)", omega));

end

disp("Done. Generated 8 error figures: (omega=0.1 and omega=1) ¡Á (linear/nonlinear) ¡Á (e_s, e_phi).");

end
