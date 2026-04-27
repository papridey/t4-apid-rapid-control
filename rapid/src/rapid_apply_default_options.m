function opts = rapid_apply_default_options(opts)
    opts = rapid_set_default(opts, 'dt_seconds', 0.1);
    opts = rapid_set_default(opts, 'sim_hours', 80);
    opts = rapid_set_default(opts, 'do_plots', true);
    opts = rapid_set_default(opts, 'verbose', true);
    opts = rapid_set_default(opts, 'do_save_pdf', false);
    opts = rapid_set_default(opts, 'output_dir', pwd);
    opts = rapid_set_default(opts, 'rng_seed', 1);

    opts = rapid_set_default(opts, 'robust_M', 5);
    opts = rapid_set_default(opts, 'robust_lambda_cvar', 0.6);
    opts = rapid_set_default(opts, 'robust_cvar_q', 0.80);

    opts = rapid_set_default(opts, 'param_scale_default', 0.08);
    opts = rapid_set_default(opts, 'kappa_scale', 0.10);
    opts = rapid_set_default(opts, 'K2_scale', 0.10);
    opts = rapid_set_default(opts, 'd4_bias', 0.01);
    opts = rapid_set_default(opts, 'd4_amp', 0.01);
    opts = rapid_set_default(opts, 'act_gain_scale', 0.10);
    opts = rapid_set_default(opts, 'delay_seconds', 5);
    opts = rapid_set_default(opts, 'jitter_seconds', 2);
end
