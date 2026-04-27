function [OUT_coarse, OUT_fine] = exp_actuator_stepsize_comparison(opts)
%EXP_ACTUATOR_STEPSIZE_COMPARISON
% Compare APID simulations under two actuator/simulation time resolutions:
%   dt = 0.1 s
%   dt = 0.1/12 s
%
% This script runs the same APID controller twice and generates:
%   1) a two-panel summary comparison of measured T4 and supplied EF,
%   2) a zoomed EF comparison,
%   3) an optional MAT file with both outputs.
%
% Usage:
%   exp_actuator_stepsize_comparison
%   exp_actuator_stepsize_comparison(struct('T4_des',25))
%
% Optional fields in opts:
%   .T4_des          (default: 25)
%   .dt_coarse       (default: 0.1)
%   .dt_fine         (default: 0.1/12)
%   .output_dir      (default: fullfile(pwd,'results','stepsize_comparison'))
%   .save_plot       (default: true)
%   .save_mat        (default: true)
%   .verbose         (default: true)
%   .sim_hours       (default: 80)
%   .plot_window_hr  (default: [0 80])
%   .zoom_window_hr  (default: [12 24])

    if nargin < 1
        opts = struct();
    end

    opts = set_default(opts, 'T4_des', 25);
    opts = set_default(opts, 'dt_coarse', 0.1);
    opts = set_default(opts, 'dt_fine', 0.1/12);
    opts = set_default(opts, 'output_dir', fullfile(pwd, 'results', 'stepsize_comparison'));
    opts = set_default(opts, 'save_plot', true);
    opts = set_default(opts, 'save_mat', true);
    opts = set_default(opts, 'verbose', true);
    opts = set_default(opts, 'sim_hours', 80);
    opts = set_default(opts, 'plot_window_hr', [0 80]);
    opts = set_default(opts, 'zoom_window_hr', [12 24]);

    if ~exist(opts.output_dir, 'dir')
        mkdir(opts.output_dir);
    end

    ts = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));

    if opts.verbose
        fprintf('=== Actuator step-size comparison ===\n');
        fprintf('Setpoint: %g\n', opts.T4_des);
        fprintf('dt coarse: %.8g s\n', opts.dt_coarse);
        fprintf('dt fine:   %.8g s\n', opts.dt_fine);
        fprintf('Output directory:\n  %s\n\n', opts.output_dir);
    end

    % -------------------------------------------------------------
    % Run APID with coarse dt
    % -------------------------------------------------------------
    apid_opts_coarse = struct();
    apid_opts_coarse.dt_seconds = opts.dt_coarse;
    apid_opts_coarse.sim_hours  = opts.sim_hours;
    apid_opts_coarse.make_plots = false;
    apid_opts_coarse.verbose    = opts.verbose;
    apid_opts_coarse.output_dir = opts.output_dir;

    if opts.verbose
        fprintf('Running coarse-step APID...\n');
    end
    OUT_coarse = run_apid_bandlock_smooth(opts.T4_des, apid_opts_coarse);

    % -------------------------------------------------------------
    % Run APID with fine dt
    % -------------------------------------------------------------
    apid_opts_fine = struct();
    apid_opts_fine.dt_seconds = opts.dt_fine;
    apid_opts_fine.sim_hours  = opts.sim_hours;
    apid_opts_fine.make_plots = false;
    apid_opts_fine.verbose    = opts.verbose;
    apid_opts_fine.output_dir = opts.output_dir;

    if opts.verbose
        fprintf('Running fine-step APID...\n');
    end
    OUT_fine = run_apid_bandlock_smooth(opts.T4_des, apid_opts_fine);

    % -------------------------------------------------------------
    % Save MAT file
    % -------------------------------------------------------------
    if opts.save_mat
        matfile = fullfile(opts.output_dir, "stepsize_comparison_" + ts + ".mat");
        save(matfile, 'OUT_coarse', 'OUT_fine', 'opts');
        if opts.verbose
            fprintf('Saved MAT output:\n  %s\n\n', matfile);
        end
    end

    % -------------------------------------------------------------
    % Figure 1: measured T4 + EF comparison
    % -------------------------------------------------------------
    fig1 = figure('Color','w','Position',[100 100 1200 750]);

    % --- panel 1: measured T4
    ax1 = subplot(2,1,1); hold(ax1,'on');

    mask_c = ~isnan(OUT_coarse.t_meas) & ~isnan(OUT_coarse.T4_meas);
    mask_f = ~isnan(OUT_fine.t_meas)   & ~isnan(OUT_fine.T4_meas);

    t_meas_c = OUT_coarse.t_meas(mask_c) / 60;
    y_meas_c = OUT_coarse.T4_meas(mask_c);

    t_meas_f = OUT_fine.t_meas(mask_f) / 60;
    y_meas_f = OUT_fine.T4_meas(mask_f);

    plot(ax1, t_meas_c, y_meas_c, '-o', 'LineWidth', 1.5, 'MarkerSize', 4, ...
        'DisplayName', sprintf('Measured T_4, dt = %.4g s', opts.dt_coarse));
    plot(ax1, t_meas_f, y_meas_f, '-s', 'LineWidth', 1.5, 'MarkerSize', 4, ...
        'DisplayName', sprintf('Measured T_4, dt = %.4g s', opts.dt_fine));

    yline(ax1, opts.T4_des, '--k', 'LineWidth', 1.2, 'DisplayName', 'Setpoint');

    band_pct = 0.05;
    yline(ax1, opts.T4_des*(1-band_pct), ':', 'Color', [0.5 0.5 0.5], ...
        'LineWidth', 1.0, 'HandleVisibility', 'off');
    yline(ax1, opts.T4_des*(1+band_pct), ':', 'Color', [0.5 0.5 0.5], ...
        'LineWidth', 1.0, 'HandleVisibility', 'off');

    xlabel(ax1, 'Time (h)');
    ylabel(ax1, 'Measured T_4^{ext} (ng/\muL)');
    title(ax1, 'Measured tracking comparison');
    grid(ax1, 'on');
    xlim(ax1, opts.plot_window_hr);
    legend(ax1, 'Location', 'best');

    % --- panel 2: EF supplied
    ax2 = subplot(2,1,2); hold(ax2,'on');

    idxc = downsample_index(OUT_coarse.time, 6000);
    idxf = downsample_index(OUT_fine.time, 6000);

    plot(ax2, OUT_coarse.time_hr(idxc), OUT_coarse.EF_supplied_log(idxc), ...
        'LineWidth', 1.2, 'DisplayName', sprintf('EF supplied, dt = %.4g s', opts.dt_coarse));
    plot(ax2, OUT_fine.time_hr(idxf), OUT_fine.EF_supplied_log(idxf), ...
        'LineWidth', 1.2, 'DisplayName', sprintf('EF supplied, dt = %.4g s', opts.dt_fine));

    xlabel(ax2, 'Time (h)');
    ylabel(ax2, 'EF supplied');
    title(ax2, 'Burst-averaged EF comparison');
    grid(ax2, 'on');
    xlim(ax2, opts.plot_window_hr);
    legend(ax2, 'Location', 'best');

    linkaxes([ax1 ax2], 'x');

    % -------------------------------------------------------------
    % Figure 2: zoomed EF comparison
    % -------------------------------------------------------------
    fig2 = figure('Color','w','Position',[140 140 1100 350]);
    axz = axes(fig2); hold(axz,'on');

    mask_zoom_c = OUT_coarse.time_hr >= opts.zoom_window_hr(1) & OUT_coarse.time_hr <= opts.zoom_window_hr(2);
    mask_zoom_f = OUT_fine.time_hr   >= opts.zoom_window_hr(1) & OUT_fine.time_hr   <= opts.zoom_window_hr(2);

    plot(axz, OUT_coarse.time_hr(mask_zoom_c), OUT_coarse.EF_supplied_log(mask_zoom_c), ...
        'LineWidth', 1.2, 'DisplayName', sprintf('dt = %.4g s', opts.dt_coarse));
    plot(axz, OUT_fine.time_hr(mask_zoom_f), OUT_fine.EF_supplied_log(mask_zoom_f), ...
        'LineWidth', 1.2, 'DisplayName', sprintf('dt = %.4g s', opts.dt_fine));

    xlabel(axz, 'Time (h)');
    ylabel(axz, 'EF supplied');
    title(axz, sprintf('Zoomed EF comparison on [%.2f, %.2f] h', opts.zoom_window_hr(1), opts.zoom_window_hr(2)));
    grid(axz, 'on');
    xlim(axz, opts.zoom_window_hr);
    legend(axz, 'Location', 'best');

    % -------------------------------------------------------------
    % Save figures
    % -------------------------------------------------------------
    if opts.save_plot
        f1pdf = fullfile(opts.output_dir, "stepsize_comparison_summary_" + ts + ".pdf");
        f1png = fullfile(opts.output_dir, "stepsize_comparison_summary_" + ts + ".png");
        f2pdf = fullfile(opts.output_dir, "stepsize_comparison_EF_zoom_" + ts + ".pdf");
        f2png = fullfile(opts.output_dir, "stepsize_comparison_EF_zoom_" + ts + ".png");

        exportgraphics(fig1, f1pdf, 'ContentType', 'vector');
        exportgraphics(fig1, f1png, 'Resolution', 300);
        exportgraphics(fig2, f2pdf, 'ContentType', 'vector');
        exportgraphics(fig2, f2png, 'Resolution', 300);

        if opts.verbose
            fprintf('Saved figures:\n');
            fprintf('  %s\n', f1pdf);
            fprintf('  %s\n', f1png);
            fprintf('  %s\n', f2pdf);
            fprintf('  %s\n', f2png);
        end
    end
end

% =====================================================================
% local helpers
% =====================================================================

function idx = downsample_index(time_vec, nmax)
    stride = max(1, ceil(numel(time_vec) / nmax));
    idx = 1:stride:numel(time_vec);
end

function s = set_default(s, name, value)
    if ~isfield(s, name) || isempty(s.(name))
        s.(name) = value;
    end
end