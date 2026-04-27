function v = clamp(x, xmin, xmax)
%CLAMP Saturate x to [xmin, xmax].

    v = min(max(x, xmin), xmax);
end
