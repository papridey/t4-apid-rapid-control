function Jrob = rapid_window_cost_ISE_robust( ...
    y_state, Kp, Ki, Kd, r_f, y_meas_used, int_e, prev_e, Ts, ...
    EF_prev, A_prev, CTRL, SCHED, t_now, scen, ROB, P)

    if ~ROB.enable || isempty(scen)
        pert = rapid_neutral_perturbation(ROB.d4_period_min);
        Jrob = rapid_window_cost_ISE_one_robust( ...
            y_state, Kp, Ki, Kd, r_f, y_meas_used, int_e, prev_e, Ts, ...
            EF_prev, A_prev, CTRL, SCHED, t_now, pert, ROB, P);
        return;
    end

    M = numel(scen);
    J = zeros(M,1);
    for i = 1:M
        J(i) = rapid_window_cost_ISE_one_robust( ...
            y_state, Kp, Ki, Kd, r_f, y_meas_used, int_e, prev_e, Ts, ...
            EF_prev, A_prev, CTRL, SCHED, t_now, scen(i), ROB, P);
    end

    Jmean = mean(J);
    q = ROB.cvar_q;
    k = max(1, ceil((1 - q) * M));
    Jsort = sort(J, 'descend');
    Jcvar = mean(Jsort(1:k));
    Jrob = (1 - ROB.lambda_cvar) * Jmean + ROB.lambda_cvar * Jcvar;
end
