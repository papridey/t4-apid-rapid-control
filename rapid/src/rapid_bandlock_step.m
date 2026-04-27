function [EF_cmd_next, A_next, A_basal] = rapid_bandlock_step(e_true, A_hold, A_basal, CTRL)
    A_basal = A_basal + CTRL.eta_basal * e_true;
    A_basal = rapid_clamp(A_basal, CTRL.A_basal_min, CTRL.A_basal_max);

    A_next = A_basal + CTRL.lock_kA * e_true;
    A_next = rapid_clamp(A_next, CTRL.A_min, CTRL.A_max);
    A_next = rapid_rate_limit(A_next, A_hold, CTRL.dA_max);
    EF_cmd_next = rapid_map_A_to_EF(A_next, CTRL.thresh, CTRL.kA, CTRL.EF_min, CTRL.EF_max);
end
