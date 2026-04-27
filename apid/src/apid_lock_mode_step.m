function [EF_cmd_next, A_next, state] = apid_lock_mode_step(state, e_true, ctrl)
%APID_LOCK_MODE_STEP Basal-hold update used inside the band-lock region.

    state.A_basal = state.A_basal + ctrl.eta_basal * e_true;
    state.A_basal = clamp(state.A_basal, ctrl.A_basal_min, ctrl.A_basal_max);

    A_next = state.A_basal + ctrl.lock_kA * e_true;
    A_next = clamp(A_next, ctrl.A_min, ctrl.A_max);
    A_next = rate_limit(A_next, state.A_hold, ctrl.dA_max);

    EF_cmd_next = map_A_to_EF(A_next, ctrl.thresh, ctrl.kA, ctrl.EF_min, ctrl.EF_max);
end
