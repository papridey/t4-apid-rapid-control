function J = window_cost_ISE_apid( ...
    y_state, Kp, Ki, Kd, T4_des, y_meas, prev_y, dmeas_f_prev, int_e, Ts, ...
    EF_prev, A_prev, A_basal, first_band_hit, EF_min, EF_max, EF_rate, Kb_aw, ...
    thresh, kA, A_min, A_max, dA_max, A_filter_fac, T_total, T_cycle, T_on, ...
    t_now, pulseDuty, t5, t6, rhoA, beta_ov, P, alpha_d)
%WINDOW_COST_ISE_APID One-window predictive cost used by APID.

    e0 = T4_des - y_meas;
    dmeas = (y_meas - prev_y) / Ts;
    dmeas_f = (1 - alpha_d) * dmeas_f_prev + alpha_d * dmeas;

    if abs(e0) < 0.5
        Ki_eff = 1.25 * Ki;
    else
        Ki_eff = Ki;
    end

    Itrial = int_e + e0 * Ts;
    Itrial = clamp(Itrial, -2.0e6, 2.0e6);

    der_term = -Kd * dmeas_f;
    EF_unsat = Kp * e0 + Ki_eff * Itrial + der_term;
    EF_rl    = rate_limit(EF_unsat, EF_prev, EF_rate);
    EF_cmd   = clamp(EF_rl, EF_min, EF_max);

    aw_term = Kb_aw * (EF_cmd - EF_unsat);
    Itrial  = Itrial + aw_term * Ts;

    A_next = map_EF_to_A(EF_cmd, thresh, kA, A_min, A_max);
    A_next = (1 - A_filter_fac) * A_next + A_filter_fac * A_prev;
    A_next = rate_limit(A_next, A_prev, dA_max);
    A_next = clamp(A_next, A_min, A_max);

    if first_band_hit
        A_next = max(A_next, 0.98 * A_basal);
    end

    EF_t_min = @(t_min) ef_avg_burst_in_window( ...
        t_min, T_on, T_cycle, T_total, t5, t6, A_next, pulseDuty);

    tspan = linspace(t_now, t_now + Ts, 120);
    odef  = @(t_min, y) ode_constant_coeff_model(t_min, y, EF_t_min, P);
    [T, Y] = ode15s(odef, tspan, y_state);
    y4 = Y(:,4);

    err = y4 - T4_des;
    pos = max(err, 0);
    neg = min(err, 0);

    J_ISE = trapz(T, beta_ov * pos.^2 + neg.^2);
    J = J_ISE + rhoA * (A_next * A_next);
end
