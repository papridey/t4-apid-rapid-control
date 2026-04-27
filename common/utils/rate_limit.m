function y = rate_limit(x, x_prev, dx_max)
%RATE_LIMIT Increment-limited update.

    y = x_prev + clamp(x - x_prev, -dx_max, dx_max);
end
