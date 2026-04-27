function state = rapid_init_controller_state(T4_des, y0, CTRL, MEAS, ~)
    state.r_f = T4_des;
    state.Kp = CTRL.Kp0;
    state.Ki = CTRL.Ki0;
    state.Kd = CTRL.Kd0;

    e0 = state.r_f - y0;
    state.int_e = 0;
    state.prev_e = e0;
    state.prev_y_used = y0;
    state.dmeas_f = 0;

    state.EF_cmd = rapid_clamp(state.Kp * e0 + state.Ki * state.int_e, CTRL.EF_min, CTRL.EF_max);
    state.A_hold = rapid_map_EF_to_A(state.EF_cmd, CTRL.thresh, CTRL.kA, CTRL.A_min, CTRL.A_max);
    if CTRL.WARM_START
        state.A_hold = max(state.A_hold, CTRL.A_warm);
    end

    state.meas_state.y_filt = NaN;
    state.meas_state.bhat   = 0.0;

    state.band_locked = false;
    state.first_band_hit = false;
    state.A_basal = state.A_hold;
end
