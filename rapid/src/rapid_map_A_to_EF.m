function EF = rapid_map_A_to_EF(A, thresh, kA, EF_min, EF_max)
    if A <= 0
        EF = 0;
    else
        EF = rapid_clamp(thresh + A / kA, EF_min, EF_max);
    end
end
