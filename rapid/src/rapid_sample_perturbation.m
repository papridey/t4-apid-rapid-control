function pert = rapid_sample_perturbation(R, d4_period_min, rs)
    if nargin < 3 || isempty(rs)
        rs = RandStream.getGlobalStream();
    end

    u_sym = @(a) -a + 2*a*rand(rs);
    u_pos = @(a) a*rand(rs);

    pert.alpha_scale   = u_sym(R.alpha_scale);
    pert.K_scale       = u_sym(R.K_scale);
    pert.gamma1_scale  = u_sym(R.gamma1_scale);
    pert.kappa_scale   = u_sym(R.kappa_scale);
    pert.K2_scale      = u_sym(R.K2_scale);
    pert.gammaI_scale  = u_sym(R.gammaI_scale);
    pert.gammaTg_scale = u_sym(R.gammaTg_scale);
    pert.gint_scale    = u_sym(R.gint_scale);
    pert.gext_scale    = u_sym(R.gext_scale);
    pert.M_scale       = u_sym(R.M_scale);
    pert.alpha1_scale  = u_sym(R.alpha1_scale);

    pert.d4_bias       = u_sym(R.d4_bias);
    pert.d4_amp        = u_pos(R.d4_amp);
    pert.d4_phase      = 2*pi*rand(rs);
    pert.d4_period_min = d4_period_min;

    pert.act_gain      = u_sym(R.act_gain);
    pert.delay_min     = max(0, u_sym(R.delay_min));
    pert.jitter_min    = u_pos(R.jitter_min);

    pert.meas_sigma = 0;
    pert.meas_bias  = 0;
    pert.noise_seed = 0;
end
