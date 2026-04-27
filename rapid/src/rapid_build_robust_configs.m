function [PERT_TRUE, ROB] = rapid_build_robust_configs(opts, MEAS, CONST)
    PERT_TRUE.range.alpha_scale   = opts.param_scale_default;
    PERT_TRUE.range.K_scale       = opts.param_scale_default;
    PERT_TRUE.range.gamma1_scale  = opts.param_scale_default;
    PERT_TRUE.range.kappa_scale   = opts.kappa_scale;
    PERT_TRUE.range.K2_scale      = opts.K2_scale;
    PERT_TRUE.range.gammaI_scale  = opts.param_scale_default;
    PERT_TRUE.range.gammaTg_scale = opts.param_scale_default;
    PERT_TRUE.range.gint_scale    = opts.param_scale_default;
    PERT_TRUE.range.gext_scale    = opts.param_scale_default;
    PERT_TRUE.range.M_scale       = opts.param_scale_default;
    PERT_TRUE.range.alpha1_scale  = opts.param_scale_default;

    PERT_TRUE.range.d4_bias    = opts.d4_bias;
    PERT_TRUE.range.d4_amp     = opts.d4_amp;
    PERT_TRUE.d4_period_min    = 6 * 60;
    PERT_TRUE.range.act_gain   = opts.act_gain_scale;
    PERT_TRUE.range.delay_min  = CONST.toMin(opts.delay_seconds);
    PERT_TRUE.range.jitter_min = CONST.toMin(opts.jitter_seconds);

    ROB.enable        = true;
    ROB.M             = opts.robust_M;
    ROB.cvar_q        = opts.robust_cvar_q;
    ROB.lambda_cvar   = opts.robust_lambda_cvar;
    ROB.range         = PERT_TRUE.range;
    ROB.d4_period_min = PERT_TRUE.d4_period_min;
    ROB.meas_in_cost  = true;
    ROB.meas_sigma    = MEAS.sigma;
    ROB.meas_bias     = MEAS.bias_true;
    ROB.noise_Ngrid   = 120;
end
