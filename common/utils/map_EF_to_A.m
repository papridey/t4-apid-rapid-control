function A = map_EF_to_A(EF_cmd, thresh, kA, A_min, A_max)
%MAP_EF_TO_A Map EF command to amplitude command.

    if EF_cmd < thresh
        A = 0;
    else
        A = clamp(kA * (EF_cmd - thresh), A_min, A_max);
    end
end
