function [EF_cmd_next, A_next, state] = apid_nobandlock_adaptation_step( ...
    y_state, yk, timing, ctrl, P, state, u_prev, t_now)
%APID_NOBANDLOCK_ADAPTATION_STEP One APID-no-band-lock gain and command update.

    e_meas = state.r_f - yk;

    epsKp = max(ctrl.rel_epsKp * max(state.Kp, 1e-6), 1e-6);
    epsKi = max(ctrl.rel_epsKi * max(state.Ki, 1e-9), 1e-9);
    epsKd = max(ctrl.rel_epsKd * max(state.Kd, 1e-6), 1e-6);

    grad_Kp = fd_grad(@(x) window_cost_ISE_apid_nobandlock( ...
        y_state, x, state.Ki, state.Kd, state.r_f, yk, state.int_e, state.prev_e, timing.Ts, u_prev, state.A_hold, ctrl, timing, t_now, P), state.Kp, epsKp);

    grad_Ki = fd_grad(@(x) window_cost_ISE_apid_nobandlock( ...
        y_state, state.Kp, x, state.Kd, state.r_f, yk, state.int_e, state.prev_e, timing.Ts, u_prev, state.A_hold, ctrl, timing, t_now, P), state.Ki, epsKi);

    grad_Kd = fd_grad(@(x) window_cost_ISE_apid_nobandlock( ...
        y_state, state.Kp, state.Ki, x, state.r_f, yk, state.int_e, state.prev_e, timing.Ts, u_prev, state.A_hold, ctrl, timing, t_now, P), state.Kd, epsKd);

    gN = max(1.0, sqrt(grad_Kp^2 + grad_Ki^2 + grad_Kd^2));

    dKp = clamp(-ctrl.alpha_p  * grad_Kp / gN, -ctrl.dKp_max, ctrl.dKp_max);
    dKi = clamp(-ctrl.alpha_i  * grad_Ki / gN, -ctrl.dKi_max, ctrl.dKi_max);
    dKd = clamp(-ctrl.alpha_d_ * grad_Kd / gN, -ctrl.dKd_max, ctrl.dKd_max);

    Kp_new = clamp(state.Kp + dKp, ctrl.Kp_bounds(1), ctrl.Kp_bounds(2));
    Ki_new = clamp(state.Ki + dKi, ctrl.Ki_bounds(1), ctrl.Ki_bounds(2));
    Kd_new = clamp(state.Kd + dKd, ctrl.Kd_bounds(1), ctrl.Kd_bounds(2));

    state.Kp = (1 - ctrl.smooth_gain) * state.Kp + ctrl.smooth_gain * Kp_new;
    state.Ki = (1 - ctrl.smooth_gain) * state.Ki + ctrl.smooth_gain * Ki_new;
    state.Kd = (1 - ctrl.smooth_gain) * state.Kd + ctrl.smooth_gain * Kd_new;

    flip = (sign(e_meas) ~= 0) && (sign(state.prev_e) ~= 0) && (sign(e_meas) ~= sign(state.prev_e));
    if flip
        state.int_e = 0.5 * state.int_e;
        state.Kp    = state.Kp * ctrl.flip_backoff;
    end

    if abs(e_meas) < 0.5
        Ki_eff = 1.25 * state.Ki;
    else
        Ki_eff = state.Ki;
    end

    state.int_e = (1 - ctrl.I_leak) * state.int_e + e_meas * timing.Ts;
    state.int_e = clamp(state.int_e, -ctrl.I_max, ctrl.I_max);

    der_term = -state.Kd * state.dmeas_f;

    EF_unsat    = state.Kp * e_meas + Ki_eff * state.int_e + der_term;
    EF_cmd_raw  = rate_limit(EF_unsat, u_prev, ctrl.EF_rate);
    EF_cmd_next = clamp(EF_cmd_raw, ctrl.EF_min, ctrl.EF_max);

    aw_term = ctrl.Kb_aw * (EF_cmd_next - EF_unsat);
    state.int_e = state.int_e + aw_term * timing.Ts;

    A_pid  = map_EF_to_A(EF_cmd_next, ctrl.thresh, ctrl.kA, ctrl.A_min, ctrl.A_max);
    A_pid  = (1 - ctrl.A_filter_fac) * A_pid + ctrl.A_filter_fac * state.A_hold;
    A_next = rate_limit(A_pid, state.A_hold, ctrl.dA_max);
    A_next = clamp(A_next, ctrl.A_min, ctrl.A_max);
end
