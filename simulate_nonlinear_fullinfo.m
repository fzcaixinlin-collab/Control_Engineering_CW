function out = simulate_nonlinear_fullinfo(sys, gains, cfg)
% SIMULATE_NONLINEAR_FULLINFO  (B3)
% Nonlinear model from A1:
%   M sdd + F sd - u = d1(t)
%   phidd - (g/L) sin(phi) + (1/L) sdd cos(phi) = 0
%
% Use the SAME full-information control law from B1:
%   u = -K x + L d,  d=[d1; d2; d2dot]
% d1(t) imposed as square wave; (d2,d2dot) from exosystem.

C = sys.C; S = sys.S;
K = gains.K; Lgain = gains.L;

M = cfg.M; Lp = cfg.L; F = cfg.F; g = cfg.g;

% initial conditions
d0 = [0; cfg.d2_0; cfg.d2dot_0];
z0 = [cfg.x0; d0];

ode = @(t,z) dyn(t,z);
opts = odeset('RelTol',1e-7,'AbsTol',1e-9);
[t,z] = ode45(ode, [0 cfg.tEnd], z0, opts);

x = z(:,1:4).';
d = z(:,5:7).';

y  = C*x;
u  = zeros(1,numel(t));
d2 = d(2,:);

for k = 1:numel(t)
    u(k) = -K*x(:,k) + Lgain*d(:,k);
end

out.t = t;
out.y = y;
out.u = u;
out.d2 = d2;

    function dz = dyn(t,z)
        x = z(1:4);
        d = z(5:7);

        % enforce square-wave disturbance
        d1 = d1_square(t, cfg.d1_amp, cfg.d1_period);
        d(1) = d1;

        % control (full info)
        u = -K*x + Lgain*d;

        % unpack states
        sdot = x(2);
        phi  = x(3);
        phid = x(4);

        % cart equation: M sdd + F sdot - u = d1  ->  sdd = (u - F sdot + d1)/M
        sdd = (u - F*sdot + d1)/M;

        % pendulum: phidd = (g/L) sin(phi) - (1/L) sdd cos(phi)
        phidd = (g/Lp)*sin(phi) - (1/Lp)*sdd*cos(phi);

        xdot = [sdot;
                sdd;
                phid;
                phidd];

        ddot = S*d;
        dz = [xdot; ddot];
    end
end
