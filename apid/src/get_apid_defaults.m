function opts = get_apid_defaults(opts)
%GET_APID_DEFAULTS Set default APID run options.

    opts = set_default(opts, 'dt_seconds', 0.1);
    opts = set_default(opts, 'sim_hours', 80);
    opts = set_default(opts, 'make_plots', true);
    opts = set_default(opts, 'verbose', true);
    opts = set_default(opts, 'do_save_pdf', false);
    opts = set_default(opts, 'output_dir', pwd);
end
