function run_w10_errors_only()
% Only run omega=10, only plot error signals e(t).
% Avoid memory blow-up by using fixed tspan grid.

clc; close all;

% ---------- parameters (keep consistent with your original values) ----------
cfg.M = 1;
cfg.L = 1;
cfg.F = 1;
cfg.g = 9.81;

cfg.alpha     = 1;
omega_list = [0.1, 1];

cfg.d1_amp    = 0.5;
cfg.d1_period = 50;

cfg.x0 = zeros(4,1);   % x(0)=0
cfg.d2_0    = 0;
cfg.d2dot_0 = cfg.alpha * cfg.omega;

% IMPORTANT: shorten horizon for omega=10 to avoid huge data.
% This is consistent: omega=10 has period ~0.63s, 30s already shows behaviour well.
cfg.tEnd = 30;

% Fixed output grid (limits number of stored points)
dt = 1e-3;                 % 0.001s resolution (30,001 points) good enough
tspan = 0:dt:cfg.tEnd;

% ---------- design full-info regulator for this omega ----------
[sys, sol, gains] = design_fullinfo_regulator(cfg); %#ok<ASGLU>

% ---------- B2 linear: compute e_s(t)=s-d2, e_phi(t)=phi ----------
outLin = simulate_linear_fullinfo_fixedgrid(sys, gains, cfg, tspan);
plot_error_signals(outLin, cfg, "B2 Linear (omega=10)");

% ---------- B3 nonlinear: compute e_s(t)=s-d2, e_phi(t)=phi ----------
outNL  = simulate_nonlinear_fullinfo_fixedgrid(sys, gains, cfg, tspan);
plot_error_signals(outNL, cfg, "B3 Nonlinear (omega=10)");

end
