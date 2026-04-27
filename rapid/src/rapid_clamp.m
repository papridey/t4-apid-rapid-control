function v = rapid_clamp(x, xmin, xmax)
    v = min(max(x, xmin), xmax);
end
