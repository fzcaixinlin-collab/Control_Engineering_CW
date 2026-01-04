function out = simulate_nonlinear_fullinfo_fixedgrid(sys, gains, cfg, tspan)
% Nonlinear simulation (A1 model) with fixed output grid tspan.
% Uses ode15s by default to handle stiffness at omega=10.

S = sys.S;
K = gains.K; Lgain = gains.L;

M = cfg.M; Lp = cfg.L; F = cfg.F; g = cfg.g;

d0 = [0; cfg.d2_0; cfg.d2dot_0];
z0 = [cfg.x0; d0];

ode = @(t,z) dyn(t,z);
opts = odeset('RelTol',1e-6,'AbsTol',1e-8);

% Use ode15s to reduce step explosion at high frequency
[t,z] = ode15s(ode, tspan, z0, opts);
% If you prefer ode45, replace the line above with:
% [t,z] = ode45(ode, tspan, z0, opts);

x = z(:,1:4).';
d = z(:,5:7).';

s   = x(1,:);
phi = x(3,:);
d2  = d(2,:);

e_s   = s - d2;
e_phi = phi;

out.t = t;
out.e_s = e_s;
out.e_phi = e_phi;

    function dz = dyn(t,z)
        x = z(1:4);
        d = z(5:7);

        d1 = d1_square(t, cfg.d1_amp, cfg.d1_period);
        d(1) = d1;

        u = -K*x + Lgain*d;

        sdot = x(2);
        phi  = x(3);
        phid = x(4);

        % A1 nonlinear dynamics
        sdd   = (u - F*sdot + d1)/M;
        phidd = (g/Lp)*sin(phi) - (1/Lp)*sdd*cos(phi);

        xdot = [sdot; sdd; phid; phidd];
        ddot = S*d;

        dz = [xdot; ddot];
    end
end
