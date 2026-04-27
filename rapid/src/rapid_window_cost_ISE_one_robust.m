function J = rapid_window_cost_ISE_one_robust( ...
    y_state, Kp, Ki, Kd, r_f, y_meas_used, int_e, prev_e, Ts, ...
    EF_prev, A_prev, CTRL, SCHED, t_now, pert, ROB, P)

    e0    = r_f - y_meas_used;
    der_e = (e0 - prev_e) / Ts;
    Itrial = int_e + e0 * Ts;

    EF_unsat = Kp * e0 + Ki * Itrial + Kd * der_e;
    EF_rl    = rapid_rate_limit(EF_unsat, EF_prev, CTRL.EF_rate);
    EF_cmd   = rapid_clamp(EF_rl, CTRL.EF_min, CTRL.EF_max);

    A_next = rapid_map_EF_to_A(EF_cmd, CTRL.thresh, CTRL.kA, CTRL.A_min, CTRL.A_max);
    A_next = (1 - CTRL.A_filter_fac) * A_next + CTRL.A_filter_fac * A_prev;
    A_next = rapid_rate_limit(A_next, A_prev, CTRL.dA_max);
    A_next = rapid_clamp(A_next, CTRL.A_min, CTRL.A_max);

    A_applied = rapid_clamp(A_next * (1 + pert.act_gain), CTRL.A_min, CTRL.A_max);
    delay_min = max(0, pert.delay_min);
    jitter_min = pert.jitter_min;

    EF_t_min = @(t_min) rapid_ef_avg_burst_in_window_delay( ...
        t_min, SCHED.T_on, SCHED.T_cycle, SCHED.T_total, SCHED.t5, SCHED.t6, ...
        A_applied, SCHED.pulseDuty, delay_min, jitter_min);

    tspan = linspace(t_now, t_now + Ts, ROB.noise_Ngrid);
    odef  = @(t_min, y) rapid_ode_perturbed_model(t_min, y, EF_t_min, pert, P);
    [T, Y] = ode15s(odef, tspan, y_state);
    y4_true = Y(:,4);

    if isfield(pert, 'meas_sigma') && pert.meas_sigma > 0
        if isfield(pert, 'noise_seed') && ~isempty(pert.noise_seed) && pert.noise_seed ~= 0
            rs_noise = RandStream('mt19937ar', 'Seed', pert.noise_seed);
            noise = randn(rs_noise, size(y4_true));
        else
            noise = randn(size(y4_true));
        end
        y4_cost = y4_true + pert.meas_bias + pert.meas_sigma * noise;
    else
        y4_cost = y4_true;
    end

    r = r_f * ones(size(y4_cost));
    err = y4_cost - r;
    pos = max(err, 0);
    neg = min(err, 0);

    J_ISE = trapz(T, CTRL.beta_ov * pos.^2 + neg.^2);
    J = J_ISE + CTRL.rhoA * (A_next * A_next);
end
