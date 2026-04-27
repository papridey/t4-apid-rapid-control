function [A_now, EF_time_k_min] = rapid_build_window_input(A_hold, pert_true, SCHED, CTRL)
    if A_hold <= 0
        A_now = 0;
        EF_time_k_min = @(t_min) 0.0;
        return;
    end

    A_now = A_hold;
    A_applied = rapid_clamp(A_hold * (1 + pert_true.act_gain), CTRL.A_min, CTRL.A_max);
    delay_min = max(0, pert_true.delay_min);
    jitter_min = pert_true.jitter_min;

    EF_time_k_min = @(t_min) rapid_ef_avg_burst_in_window_delay( ...
        t_min, SCHED.T_on, SCHED.T_cycle, SCHED.T_total, SCHED.t5, SCHED.t6, ...
        A_applied, SCHED.pulseDuty, delay_min, jitter_min);
end
