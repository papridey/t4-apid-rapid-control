function [Kp, Ki, Kd, int_e, EF_cmd_next, A_next] = rapid_adaptive_pid_step( ...
    y_state, Kp, Ki, Kd, r_f, y_used, int_e, prev_e, dmeas_f, SCHED, t_now, EF_cmd, A_hold, ...
    first_band_hit, A_basal, CTRL, scen, ROB, P, e_meas)

    epsKp = max(CTRL.rel_epsKp * max(Kp, 1e-6), 1e-6);
    epsKi = max(CTRL.rel_epsKi * max(Ki, 1e-9), 1e-9);
    epsKd = max(CTRL.rel_epsKd * max(Kd, 1e-6), 1e-6);

    costfun = @(kp,ki,kd) rapid_window_cost_ISE_robust( ...
        y_state, kp, ki, kd, r_f, y_used, int_e, prev_e, SCHED.Ts, ...
        EF_cmd, A_hold, CTRL, SCHED, t_now, scen, ROB, P);

    grad_Kp = rapid_fd_grad(@(x) costfun(x,Ki,Kd), Kp, epsKp);
    grad_Ki = rapid_fd_grad(@(x) costfun(Kp,x,Kd), Ki, epsKi);
    grad_Kd = rapid_fd_grad(@(x) costfun(Kp,Ki,x), Kd, epsKd);

    gN = max(1.0, sqrt(grad_Kp^2 + grad_Ki^2 + grad_Kd^2));

    dKp = rapid_clamp(-CTRL.alpha_p      * grad_Kp / gN, -CTRL.dKp_max, CTRL.dKp_max);
    dKi = rapid_clamp(-CTRL.alpha_i      * grad_Ki / gN, -CTRL.dKi_max, CTRL.dKi_max);
    dKd = rapid_clamp(-CTRL.alpha_d_gain * grad_Kd / gN, -CTRL.dKd_max, CTRL.dKd_max);

    Kp_new = rapid_clamp(Kp + dKp, CTRL.Kp_bounds(1), CTRL.Kp_bounds(2));
    Ki_new = rapid_clamp(Ki + dKi, CTRL.Ki_bounds(1), CTRL.Ki_bounds(2));
    Kd_new = rapid_clamp(Kd + dKd, CTRL.Kd_bounds(1), CTRL.Kd_bounds(2));

    Kp = (1 - CTRL.smooth_gain) * Kp + CTRL.smooth_gain * Kp_new;
    Ki = (1 - CTRL.smooth_gain) * Ki + CTRL.smooth_gain * Ki_new;
    Kd = (1 - CTRL.smooth_gain) * Kd + CTRL.smooth_gain * Kd_new;

    flip = (sign(e_meas) ~= 0) && (sign(prev_e) ~= 0) && (sign(e_meas) ~= sign(prev_e));
    if flip
        int_e = 0.5 * int_e;
        Kp = Kp * CTRL.flip_backoff;
    end

    if abs(e_meas) < 0.5
        Ki_eff = 1.25 * Ki;
    else
        Ki_eff = Ki;
    end

    int_e = (1 - CTRL.I_leak) * int_e + e_meas * SCHED.Ts;
    int_e = rapid_clamp(int_e, -CTRL.I_max, CTRL.I_max);

    der_term = -Kd * dmeas_f;
    EF_unsat   = Kp * e_meas + Ki_eff * int_e + der_term;
    EF_cmd_raw = rapid_rate_limit(EF_unsat, EF_cmd, CTRL.EF_rate);
    EF_cmd_next = rapid_clamp(EF_cmd_raw, CTRL.EF_min, CTRL.EF_max);

    aw_term = CTRL.Kb_aw * (EF_cmd_next - EF_unsat);
    int_e   = int_e + aw_term * SCHED.Ts;
    int_e   = rapid_clamp(int_e, -CTRL.I_max, CTRL.I_max);

    A_pid  = rapid_map_EF_to_A(EF_cmd_next, CTRL.thresh, CTRL.kA, CTRL.A_min, CTRL.A_max);
    A_pid  = (1 - CTRL.A_filter_fac) * A_pid + CTRL.A_filter_fac * A_hold;
    A_next = rapid_rate_limit(A_pid, A_hold, CTRL.dA_max);
    A_next = rapid_clamp(A_next, CTRL.A_min, CTRL.A_max);

    if first_band_hit
        A_next = max(A_next, 0.98 * A_basal);
    end
end
