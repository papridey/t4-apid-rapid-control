function scen = rapid_sample_perturbations(R, d4_period_min, M, rs)
    if nargin < 4 || isempty(rs)
        rs = RandStream.getGlobalStream();
    end
    if M <= 0
        scen = struct([]);
        return;
    end
    firstPert = rapid_sample_perturbation(R, d4_period_min, rs);
    scen = repmat(firstPert, 1, M);
    for i = 2:M
        scen(i) = rapid_sample_perturbation(R, d4_period_min, rs);
    end
end
