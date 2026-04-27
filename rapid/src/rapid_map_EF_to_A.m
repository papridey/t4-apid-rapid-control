function A = rapid_map_EF_to_A(EF_cmd, thresh, kA, A_min, A_max)
    if EF_cmd < thresh
        A = 0;
    else
        A = rapid_clamp(kA * (EF_cmd - thresh), A_min, A_max);
    end
end
