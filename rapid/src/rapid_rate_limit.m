function y = rapid_rate_limit(x, x_prev, dx_max)
    y = x_prev + rapid_clamp(x - x_prev, -dx_max, dx_max);
end
