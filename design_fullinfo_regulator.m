function [sys, sol, gains] = design_fullinfo_regulator(cfg)
% DESIGN_FULLINFO_REGULATOR  (B1)
% Linear model from A3, exosystem from A5, FBI solution from A6.
% IMPORTANT SIGN CONVENTION (MATLAB):
%   K = place(A,B,...) is for u = -Kx  -> closed-loop A - B K
% Therefore implement:
%   u = -K x + L d
% and choose:
%   L = Gamma + K Pi

M = cfg.M; Lp = cfg.L; F = cfg.F; g = cfg.g;
w = cfg.omega;

% --- Linearised system from your A3 (x=[s sdot phi phidot]^T)
A = [ 0,     1,     0, 0;
      0,  -F/M,     0, 0;
      0,     0,     0, 1;
      0,  F/(M*Lp), g/Lp, 0 ];

B = [ 0;
      1/M;
      0;
     -1/(M*Lp) ];

P1 = [0; 1/M; 0; -1/(M*Lp)];

% measured output y=[s;phi]
C = [1 0 0 0;
     0 0 1 0];

% tracking error e = s - d2 = Ce x + Q d
Ce = [1 0 0 0];
Q  = [0 -1 0];

% exosystem for d=[d1; d2; d2dot]
S  = [0    0     0;
      0    0     1;
      0  -w^2    0];

% --- A6 FBI closed-form (alpha=1 here; amplitude via initial condition)
alpha = 1;
Pi = [0,     alpha, 0;
      0,         0, alpha;
      0, -(alpha*w^2)/(g+w^2), 0;
      0,         0, -(alpha*w^2)/(g+w^2)];

Gamma = [-1, -alpha*w^2, alpha];

% --- Choose stabilising K for u = -Kx (so A-BK is Hurwitz)
poles = [-0.8, -1.0, -1.2, -1.4];
K = place(A, B, poles);

% --- With u = -Kx + Ld, the correct L is:
Lgain = Gamma + K*Pi;

% pack
sys.A = A; sys.B = B; sys.P1 = P1; sys.C = C;
sys.Ce = Ce; sys.Q = Q; sys.S = S;
sol.Pi = Pi; sol.Gamma = Gamma;
gains.K = K; gains.L = Lgain;

% --- Print B1 design results (paste-ready for report)
fprintf("B1 design (omega=%.3g):\n", w);
fprintf("Square disturbance: amp=%.2f, period=%.1f\n", cfg.d1_amp, cfg.d1_period);
fprintf("Reference: d2(t)=alpha*sin(omega t), alpha=%.1f\n", cfg.alpha);

disp("K (used in u = -Kx + Ld) = "); disp(K);
disp("L = "); disp(Lgain);

fprintf("Control law implemented in MATLAB:\n");
fprintf("  u(t) = -K x(t) + L d(t),   d(t) = [d1(t); d2(t); d2dot(t)]\n");
fprintf("and d1(t) is imposed as the square wave.\n");

end
