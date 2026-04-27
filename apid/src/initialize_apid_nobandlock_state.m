function state = initialize_apid_nobandlock_state(T4_des, y0, timing, ctrl)
%INITIALIZE_APID_NOBANDLOCK_STATE Initialize APID-no-band-lock controller state.

    state.r_f = T4_des;
    e0 = state.r_f - y0;

    state.int_e   = 0;
    state.prev_e  = e0;
    state.prev_y  = y0;
    state.dmeas_f = 0;

    state.Kp = ctrl.Kp0;
    state.Ki = ctrl.Ki0;
    state.Kd = ctrl.Kd0;

    state.EF_cmd = clamp(state.Kp * e0 + state.Ki * state.int_e, ctrl.EF_min, ctrl.EF_max);
    state.A_hold = map_EF_to_A(state.EF_cmd, ctrl.thresh, ctrl.kA, ctrl.A_min, ctrl.A_max);

    if ctrl.WARM_START
        state.A_hold = max(state.A_hold, ctrl.A_warm);
    end
end
