function J = window_cost_ISE_apid_nobandlock( ...
    y_state, Kp, Ki, Kd, r_f, y_meas, int_e, prev_e, Ts, EF_prev, A_prev, ctrl, timing, t_now, P)
%WINDOW_COST_ISE_APID_NOBANDLOCK One-window predictive cost for APID without band-lock.

    e0 = r_f - y_meas;
    der_e = (e0 - prev_e) / Ts;
    Itrial = int_e + e0 * Ts;
    Itrial = clamp(Itrial, -ctrl.I_max, ctrl.I_max);

    if abs(e0) < 0.5
        Ki_eff = 1.25 * Ki;
    else
        Ki_eff = Ki;
    end

    EF_unsat = Kp * e0 + Ki_eff * Itrial + Kd * der_e;
    EF_rl    = rate_limit(EF_unsat, EF_prev, ctrl.EF_rate);
    EF_cmd   = clamp(EF_rl, ctrl.EF_min, ctrl.EF_max);

    aw_term = ctrl.Kb_aw * (EF_cmd - EF_unsat);
    Itrial  = clamp(Itrial + aw_term * Ts, -ctrl.I_max, ctrl.I_max);

    A_next = map_EF_to_A(EF_cmd, ctrl.thresh, ctrl.kA, ctrl.A_min, ctrl.A_max);
    A_next = (1 - ctrl.A_filter_fac) * A_next + ctrl.A_filter_fac * A_prev;
    A_next = rate_limit(A_next, A_prev, ctrl.dA_max);
    A_next = clamp(A_next, ctrl.A_min, ctrl.A_max);

    EF_t_min = @(t_min) ef_avg_burst_in_window( ...
        t_min, timing.T_on, timing.T_cycle, timing.T_total, ...
        timing.t5, timing.t6, A_next, timing.pulseDuty);

    tspan = linspace(t_now, t_now + Ts, 120);
    odef  = @(t_min, y) ode_constant_coeff_model(t_min, y, EF_t_min, P);
    [T, Y] = ode15s(odef, tspan, y_state);
    y4 = Y(:,4);

    err = y4 - r_f;
    pos = max(err, 0);
    neg = min(err, 0);

    J_ISE = trapz(T, ctrl.beta_ov * pos.^2 + neg.^2);
    J = J_ISE + ctrl.rhoA * (A_next * A_next);
end
