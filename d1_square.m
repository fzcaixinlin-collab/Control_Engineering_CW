function d1 = d1_square(t, amp, period)
% D1_SQUARE  Square wave disturbance (+/- amp).
d1 = amp * sign(sin(2*pi*t/period));
end
