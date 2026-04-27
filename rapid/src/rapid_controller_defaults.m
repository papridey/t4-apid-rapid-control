function CTRL = rapid_controller_defaults(T4_des, SCHED)
    CTRL.T4_des = T4_des;
    CTRL.EF_min = 0;
    CTRL.EF_max = 100;
    CTRL.EF_rate = 35;

    CTRL.thresh = 1e-4;
    CTRL.A_min  = 0.00;
    CTRL.A_max  = 0.10;
    CTRL.kA     = 0.20;

    CTRL.A_warm       = 0.01;
    CTRL.WARM_START   = true;
    CTRL.dA_max       = 0.05;
    CTRL.A_filter_fac = 0.0;

    CTRL.tau_ref = SCHED.Ts;

    CTRL.Kp0 = 1.5e-3;
    CTRL.Ki0 = 5e-6;
    CTRL.Kd0 = 1.0e-6;

    CTRL.Kp_bounds = [1e-6, 1e-1];
    CTRL.Ki_bounds = [1e-9, 1e-2];
    CTRL.Kd_bounds = [0.0,  1e-1];

    CTRL.Kb_aw  = 0.35;
    CTRL.I_max  = 2.0e6;
    CTRL.I_leak = 0.0;
    CTRL.alpha_d = 0.25;

    CTRL.rel_epsKp = 0.10;
    CTRL.rel_epsKi = 0.10;
    CTRL.rel_epsKd = 0.10;

    CTRL.alpha_p = 5e-2;
    CTRL.alpha_i = 5e-3;
    CTRL.alpha_d_gain = 1e-2;

    CTRL.dKp_max = 2e-4;
    CTRL.dKi_max = 2e-7;
    CTRL.dKd_max = 2e-5;

    CTRL.smooth_gain  = 0.9;
    CTRL.rhoA         = 5e-4;
    CTRL.beta_ov      = 3;
    CTRL.flip_backoff = 0.6;

    CTRL.band_pct_in  = 0.10;
    CTRL.band_pct_out = 0.12;
    CTRL.lock_kA      = 8e-4;
    CTRL.eta_basal    = 3e-4;
    CTRL.A_basal_min  = CTRL.A_min;
    CTRL.A_basal_max  = CTRL.A_max;
end
