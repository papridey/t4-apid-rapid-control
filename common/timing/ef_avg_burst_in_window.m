function ef = ef_avg_burst_in_window(t_min, T_on, T_cycle, T_total, t5, t6, A, pulseDuty)
%EF_AVG_BURST_IN_WINDOW Average EF delivered during bursts within an "ON" window.
%
% This matches the APID timing model: within each window of length T_cycle,
% the first T_on minutes are active. Inside the active portion, bursts of
% length t5 repeat with an inter-burst gap t6.

    if t_min < 0 || t_min > T_total
        ef = 0;
        return;
    end

    tau_win = mod(t_min, T_cycle);
    if tau_win >= T_on
        ef = 0;
        return;
    end

    Tb = t5 + t6;
    xi = mod(tau_win, Tb);
    in_burst = (xi < t5);

    ef = A * pulseDuty * double(in_burst);
end
