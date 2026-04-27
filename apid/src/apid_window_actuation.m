function [A_now, EF_time_k_min] = apid_window_actuation(A_hold, timing, ctrl)
%APID_WINDOW_ACTUATION Build held burst-averaged EF for the current window.

    if A_hold <= 0
        A_now = 0;
        EF_time_k_min = @(t_min) 0.0;
    else
        A_now = A_hold;
        EF_time_k_min = @(t_min) ef_avg_burst_in_window( ...
            t_min, timing.T_on, timing.T_cycle, timing.T_total, ...
            timing.t5, timing.t6, A_hold, timing.pulseDuty);
    end
end
