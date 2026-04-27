function ctrl = get_apid_controller_params()
%GET_APID_CONTROLLER_PARAMS Return APID controller and adaptation parameters.

    ctrl.EF_min = 0;
    ctrl.EF_max = 100;
    ctrl.EF_rate = 25;

    ctrl.thresh = 1e-4;
    ctrl.A_min  = 0.00;
    ctrl.A_max  = 0.10;
    ctrl.kA     = 0.20;

    ctrl.A_warm       = 0.01;
    ctrl.WARM_START   = true;
    ctrl.dA_max       = 0.03;
    ctrl.A_filter_fac = 0.0;

    ctrl.Kp0 = 2.0e-3;
    ctrl.Ki0 = 5.0e-7;
    ctrl.Kd0 = 1.0e-6;

    ctrl.Kp_bounds = [1e-6, 1e-1];
    ctrl.Ki_bounds = [1e-9, 1e-2];
    ctrl.Kd_bounds = [0.0,  1e-1];

    ctrl.Kb_aw   = 0.35;
    ctrl.I_max   = 2.0e6;
    ctrl.I_leak  = 0.0;
    ctrl.alpha_d = 0.25;

    ctrl.rel_epsKp = 0.10;
    ctrl.rel_epsKi = 0.10;
    ctrl.rel_epsKd = 0.10;

    ctrl.alpha_p  = 5e-2;
    ctrl.alpha_i  = 5e-3;
    ctrl.alpha_d_ = 1e-2;

    ctrl.dKp_max = 2e-4;
    ctrl.dKi_max = 2e-7;
    ctrl.dKd_max = 2e-5;

    ctrl.smooth_gain  = 0.8;
    ctrl.rhoA         = 5e-4;
    ctrl.beta_ov      = 3;
    ctrl.flip_backoff = 0.6;

    ctrl.band_pct_in  = 0.05;
    ctrl.band_pct_out = 0.08;
    ctrl.lock_kA      = 5e-4;
    ctrl.eta_basal    = 2e-4;

    ctrl.A_basal_min = ctrl.A_min;
    ctrl.A_basal_max = ctrl.A_max;
end
