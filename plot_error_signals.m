function plot_error_signals(out, cfg, titleTag)
% Plot e_s(t) = s(t)-d2(t) and e_phi(t) = phi(t)

t = out.t;

figure('Name', titleTag + " - e_s");
plot(t, out.e_s, 'LineWidth', 1.1);
grid on;
xlabel('t (s)'); ylabel('e_s(t) = s(t) - d_2(t)');
title(sprintf('%s: tracking error e_s(t), omega=%.3g', titleTag, cfg.omega));

figure('Name', titleTag + " - e_phi");
plot(t, out.e_phi, 'LineWidth', 1.1);
grid on;
xlabel('t (s)'); ylabel('e_\phi(t) = \phi(t)');
title(sprintf('%s: pendulum angle (deviation), omega=%.3g', titleTag, cfg.omega));
end
