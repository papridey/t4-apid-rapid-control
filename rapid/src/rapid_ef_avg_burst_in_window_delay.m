function ef = rapid_ef_avg_burst_in_window_delay(t_min, T_on, T_cycle, T_total, t5, t6, A, pulseDuty, delay_min, jitter_min)
    t_eff = t_min - delay_min + jitter_min;
    if t_eff < 0 || t_eff > T_total
        ef = 0;
        return;
    end
    tau_win = mod(t_eff, T_cycle);
    if tau_win >= T_on
        ef = 0;
        return;
    end
    Tb = t5 + t6;
    xi = mod(tau_win, Tb);
    in_burst = (xi < t5);
    ef = A * pulseDuty * double(in_burst);
end
