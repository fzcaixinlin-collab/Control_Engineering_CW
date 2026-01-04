function plot_y_u(out, cfg, titleTag)
% PLOT_Y_U  Required by B2/B3: display plots of y(t) and u(t).
% y(t) = [s(t); phi(t)] in your A3.

t  = out.t;
y  = out.y;      % 2xN
u  = out.u;      % 1xN
d2 = out.d2;     % reference to show tracking on y1

figure('Name', titleTag + " - y(t)");
plot(t, y(1,:), 'LineWidth', 1.1); hold on;
plot(t, d2,     'LineWidth', 1.1);
plot(t, y(2,:), 'LineWidth', 1.1);
grid on;
xlabel('t (s)'); ylabel('y(t)');
legend('s(t)=y_1(t)', 'd_2(t) reference', '\phi(t)=y_2(t)', 'Location','best');
title(sprintf('%s | d1 amp=%.1f, period=%.1f, omega=%.3g', ...
    titleTag, cfg.d1_amp, cfg.d1_period, cfg.omega));

figure('Name', titleTag + " - u(t)");
plot(t, u, 'LineWidth', 1.1);
grid on;
xlabel('t (s)'); ylabel('u(t)');
title(sprintf('%s: u(t)', titleTag));
end
