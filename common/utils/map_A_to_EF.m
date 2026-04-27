function EF = map_A_to_EF(A, thresh, kA, EF_min, EF_max)
%MAP_A_TO_EF Inverse map from amplitude to EF command.

    if A <= 0
        EF = 0;
    else
        EF = clamp(thresh + A / kA, EF_min, EF_max);
    end
end
