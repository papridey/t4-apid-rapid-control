function pert = rapid_neutral_perturbation(d4_period_min)
    pert.alpha_scale   = 0;
    pert.K_scale       = 0;
    pert.gamma1_scale  = 0;
    pert.kappa_scale   = 0;
    pert.K2_scale      = 0;
    pert.gammaI_scale  = 0;
    pert.gammaTg_scale = 0;
    pert.gint_scale    = 0;
    pert.gext_scale    = 0;
    pert.M_scale       = 0;
    pert.alpha1_scale  = 0;

    pert.d4_bias       = 0;
    pert.d4_amp        = 0;
    pert.d4_phase      = 0;
    pert.d4_period_min = d4_period_min;

    pert.act_gain      = 0;
    pert.delay_min     = 0;
    pert.jitter_min    = 0;

    pert.meas_sigma = 0;
    pert.meas_bias  = 0;
    pert.noise_seed = 0;
end
