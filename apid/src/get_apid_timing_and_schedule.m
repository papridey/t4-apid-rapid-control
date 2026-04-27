function timing = get_apid_timing_and_schedule(opts)
%GET_APID_TIMING_AND_SCHEDULE Build simulation horizon and stimulation timing.

    SEC_PER_MIN = 60;
    toMin = @(sec) sec / SEC_PER_MIN;

    timing.dt   = toMin(opts.dt_seconds);
    timing.ti   = 0;
    timing.tf   = opts.sim_hours * 60;
    timing.time = timing.ti:timing.dt:timing.tf;
    timing.Nsteps = numel(timing.time);

    timing.Ts      = 120;
    r              = 0.60;
    timing.T_on    = r * timing.Ts;
    timing.T_cycle = timing.Ts;
    timing.T_total = timing.tf;

    Nw = ceil((timing.tf - timing.ti) / timing.Ts);
    timing.sample_times = timing.ti + (0:Nw) * timing.Ts;
    timing.sample_times(end) = timing.tf;

    t1 = toMin(500e-6);
    t2 = toMin(100e-6);
    t3 = toMin(500e-6);
    t4 = toMin(9.5e-4);

    Tp = t1 + t2 + t3 + t4;
    timing.pulseDuty = (t1 + t3) / Tp;

    np = 500;
    timing.t5 = np * Tp;
    timing.t6 = toMin(0.5);
end
