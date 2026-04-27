function SCHED = rapid_build_schedule(opts, CONST)
    SCHED.dt = CONST.toMin(opts.dt_seconds);
    SCHED.ti = 0;
    SCHED.tf = opts.sim_hours * 60;
    SCHED.time = SCHED.ti:SCHED.dt:SCHED.tf;
    SCHED.Nsteps = numel(SCHED.time);

    SCHED.Ts = 120;
    SCHED.r = 0.60;
    SCHED.T_on = SCHED.r * SCHED.Ts;
    SCHED.T_cycle = SCHED.Ts;

    SCHED.Nw = ceil((SCHED.tf - SCHED.ti) / SCHED.Ts);
    SCHED.T_total = SCHED.ti + SCHED.Nw * SCHED.Ts;
    SCHED.sample_times = SCHED.ti + (0:SCHED.Nw) * SCHED.Ts;
    SCHED.sample_times(end) = SCHED.tf;

    SCHED.t1 = CONST.toMin(500e-6);
    SCHED.t2 = CONST.toMin(100e-6);
    SCHED.t3 = CONST.toMin(500e-6);
    SCHED.t4 = CONST.toMin(9.5e-4);
    SCHED.Tp = SCHED.t1 + SCHED.t2 + SCHED.t3 + SCHED.t4;
    SCHED.pulseDuty = (SCHED.t1 + SCHED.t3) / SCHED.Tp;
    SCHED.np = 500;
    SCHED.t5 = SCHED.np * SCHED.Tp;
    SCHED.t6 = CONST.toMin(0.5);
end
