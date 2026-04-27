function state = update_apid_bandlock_state(state, e_true, ctrl, T4_des)
%UPDATE_APID_BANDLOCK_STATE Update entry/exit of hysteretic band-lock region.

    band_in  = ctrl.band_pct_in  * max(T4_des, 1e-8);
    band_out = ctrl.band_pct_out * max(T4_des, 1e-8);

    if ~state.band_locked && abs(e_true) <= band_in
        state.band_locked    = true;
        state.first_band_hit = true;
        state.A_basal        = state.A_hold;
        state.int_e          = 0;
    elseif state.band_locked && abs(e_true) >= band_out
        state.band_locked = false;
    end
end
