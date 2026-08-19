function RESULTS = compare_optimized_fixed_pid_vs_apid(T4_des, opts)
%COMPARE_OPTIMIZED_FIXED_PID_VS_APID Compare tuned fixed PID with APID.
%
%   RESULTS = compare_optimized_fixed_pid_vs_apid()
%   RESULTS = compare_optimized_fixed_pid_vs_apid(T4_des)
%   RESULTS = compare_optimized_fixed_pid_vs_apid(T4_des, opts)
%
% This file loads the gain vector produced by
% optimize_fixed_pid_for_setpoint.m, runs the optimized fixed PID and APID,
% computes matched metrics, and generates a revised Figure 9.
%
% Recommended location:
%   <project root>/experiments/compare_optimized_fixed_pid_vs_apid.m
%
% By default, the code calls the existing APID interface:
%
%   OUT_apid = run_apid_bandlock_smooth(T4_des)
%
% If your APID function has been modified to accept an options structure,
% set opts.apid_accepts_options = true. The code will then call
%
%   OUT_apid = run_apid_bandlock_smooth(T4_des, apid_opts)
%
% and pass the evaluation seed, horizon, noise level, and optionally the
% optimized initial gains.
%
% PAPER RUN
% ---------
%   clear; clc; close all;
%   opts = struct();
%   opts.test_seeds = 1:5;
%   opts.eval_horizon_hr = 80;
%   opts.apid_accepts_options = false;
%   RESULTS = compare_optimized_fixed_pid_vs_apid(25, opts);
%
% If APID accepts options and should start from the same tuned gains:
%
%   opts.apid_accepts_options = true;
%   opts.match_apid_initial_gains = true;
%
% Generated files:
%   optimized_fixed_vs_apid_per_seed.csv
%   optimized_fixed_vs_apid_summary.csv
%   optimized_fixed_vs_apid_trajectories.pdf/.png
%   optimized_fixed_vs_apid_metric_bars.pdf/.png
%   optimized_fixed_vs_apid_results.mat

    if nargin < 1 || isempty(T4_des)
        T4_des = 25;
    end
    if nargin < 2 || isempty(opts)
        opts = struct();
    end

    thisFile = mfilename('fullpath');
    experimentsDir = fileparts(thisFile);
    projectRoot = fileparts(experimentsDir);

    add_project_paths(projectRoot);

    opts = apply_defaults(opts, projectRoot, T4_des);

    if exist('run_pid_fixed_baseline', 'file') ~= 2
        error('run_pid_fixed_baseline.m was not found.');
    end

    if exist('run_apid_bandlock_smooth', 'file') ~= 2
        error('run_apid_bandlock_smooth.m was not found.');
    end

    if ~exist(opts.output_dir, 'dir')
        mkdir(opts.output_dir);
    end

    if ~exist(opts.tuning_file, 'file')
        error(['Optimized-gain file not found:\n%s\nRun ' ...
            'optimize_fixed_pid_for_setpoint(%g, opts) first.'], ...
            opts.tuning_file, T4_des);
    end

    loaded = load(opts.tuning_file, 'TUNE');

    if ~isfield(loaded, 'TUNE') || ...
            ~isfield(loaded.TUNE, 'Kstar')
        error('The tuning MAT file does not contain TUNE.Kstar.');
    end

    Kstar = loaded.TUNE.Kstar(:)';

    if numel(Kstar) ~= 3
        error('TUNE.Kstar must contain [Kp Ki Kd].');
    end

    fprintf('\n============================================================\n');
    fprintf('Optimized fixed PID versus APID\n');
    fprintf('Setpoint: %.6g\n', T4_des);
    fprintf('Kp* = %.12g\n', Kstar(1));
    fprintf('Ki* = %.12g\n', Kstar(2));
    fprintf('Kd* = %.12g\n', Kstar(3));
    fprintf('Evaluation seeds: %s\n', mat2str(opts.test_seeds));
    fprintf('============================================================\n');

    seeds = opts.test_seeds(:)';
    nSeeds = numel(seeds);

    fixedRows = repmat(empty_metric_row(), nSeeds, 1);
    apidRows = repmat(empty_metric_row(), nSeeds, 1);

    representativeFixed = [];
    representativeAPID = [];

    checkpointDir = fullfile(opts.output_dir, 'checkpoints');

    if ~exist(checkpointDir, 'dir')
        mkdir(checkpointDir);
    end

    for i = 1:nSeeds
        seed = seeds(i);

        checkpointFile = fullfile(checkpointDir, ...
            sprintf('fixed_vs_apid_seed_%d.mat', seed));

        reused = false;

        if opts.resume && exist(checkpointFile, 'file')
            saved = load(checkpointFile, ...
                'fixedMetric', 'apidMetric', ...
                'compactFixed', 'compactAPID', 'KstarSaved');

            if isfield(saved, 'KstarSaved') && ...
                    isequaln(saved.KstarSaved, Kstar)
                fixedMetric = saved.fixedMetric;
                apidMetric = saved.apidMetric;
                compactFixed = saved.compactFixed;
                compactAPID = saved.compactAPID;
                reused = true;

                fprintf('Reusing checkpoint for seed %d.\n', seed);
            end
        end

        if ~reused
            fprintf('\nPaired evaluation seed %d\n', seed);

            fixedOpts = opts.fixed_common;
            fixedOpts.Kp = Kstar(1);
            fixedOpts.Ki = Kstar(2);
            fixedOpts.Kd = Kstar(3);
            fixedOpts.t_end_hr = opts.eval_horizon_hr;
            fixedOpts.meas_sigma = opts.meas_sigma;
            fixedOpts.rng_seed = seed;

            rng(seed, 'twister');
            fprintf('  Running optimized fixed PID...\n');
            OUTfixed = run_pid_fixed_baseline(T4_des, fixedOpts);

            rng(seed, 'twister');
            fprintf('  Running APID...\n');
            OUTapid = call_apid(T4_des, Kstar, seed, opts);

            fixedMetric = compute_metrics_from_out( ...
                OUTfixed, "Optimized fixed PID", seed);

            apidMetric = compute_metrics_from_out( ...
                OUTapid, "APID", seed);

            compactFixed = compact_output( ...
                OUTfixed, opts.max_plot_points);

            compactAPID = compact_output( ...
                OUTapid, opts.max_plot_points);

            KstarSaved = Kstar; %#ok<NASGU>

            save(checkpointFile, ...
                'fixedMetric', 'apidMetric', ...
                'compactFixed', 'compactAPID', ...
                'KstarSaved', '-v7');

            clear OUTfixed OUTapid
        end

        fixedRows(i) = fixedMetric;
        apidRows(i) = apidMetric;

        if i == 1
            representativeFixed = compactFixed;
            representativeAPID = compactAPID;
        end
    end

    fixedTable = struct2table(fixedRows);
    apidTable = struct2table(apidRows);

    perSeed = [fixedTable; apidTable];
    perSeed = sortrows(perSeed, ...
        {'seed', 'controller'}, {'ascend', 'ascend'});

    summary = make_summary_table(fixedTable, apidTable);

    writetable(perSeed, fullfile(opts.output_dir, ...
        'optimized_fixed_vs_apid_per_seed.csv'));

    writetable(summary, fullfile(opts.output_dir, ...
        'optimized_fixed_vs_apid_summary.csv'));

    make_trajectory_figure( ...
        representativeFixed, representativeAPID, ...
        T4_des, Kstar, seeds(1), opts);

    make_metric_bar_figure( ...
        fixedTable, apidTable, opts);

    RESULTS = struct();
    RESULTS.T4_des = T4_des;
    RESULTS.Kstar = Kstar;
    RESULTS.options = opts;
    RESULTS.per_seed = perSeed;
    RESULTS.summary = summary;
    RESULTS.representative_fixed = representativeFixed;
    RESULTS.representative_apid = representativeAPID;

    save(fullfile(opts.output_dir, ...
        'optimized_fixed_vs_apid_results.mat'), ...
        'RESULTS', '-v7');

    fprintf('\nSummary:\n');
    disp(summary);

    fprintf('\nResults written to:\n%s\n', opts.output_dir);
end


function opts = apply_defaults(opts, projectRoot, T4_des)
    tag = numeric_file_tag(T4_des);

    opts = set_default(opts, 'test_seeds', 1:5);
    opts = set_default(opts, 'eval_horizon_hr', 80);
    opts = set_default(opts, 'meas_sigma', 0.0);
    opts = set_default(opts, 'fixed_common', struct());
    opts = set_default(opts, 'apid_common', struct());

    opts = set_default(opts, ...
        'apid_accepts_options', false);

    opts = set_default(opts, ...
        'match_apid_initial_gains', false);

    opts = set_default(opts, 'resume', true);
    opts = set_default(opts, 'figure_visible', true);
    opts = set_default(opts, 'save_pdf', true);
    opts = set_default(opts, 'save_png', true);
    opts = set_default(opts, 'png_resolution', 600);
    opts = set_default(opts, 'max_plot_points', 8000);

    opts = set_default(opts, 'tuning_file', ...
        fullfile(projectRoot, 'results', ...
        sprintf('fixed_pid_tuning_T4_%s', tag), ...
        sprintf('optimized_fixed_pid_T4_%s.mat', tag)));

    opts = set_default(opts, 'output_dir', ...
        fullfile(projectRoot, 'results', ...
        sprintf('optimized_fixed_vs_apid_T4_%s', tag)));
end


function opts = set_default(opts, name, value)
    if ~isfield(opts, name) || isempty(opts.(name))
        opts.(name) = value;
    end
end


function OUT = call_apid(T4_des, Kstar, seed, opts)
    if ~opts.apid_accepts_options
        OUT = run_apid_bandlock_smooth(T4_des);
        return;
    end

    apidOpts = opts.apid_common;
    apidOpts.t_end_hr = opts.eval_horizon_hr;
    apidOpts.sim_hours = opts.eval_horizon_hr;
    apidOpts.meas_sigma = opts.meas_sigma;
    apidOpts.rng_seed = seed;

    if opts.match_apid_initial_gains
        apidOpts.Kp_init = Kstar(1);
        apidOpts.Ki_init = Kstar(2);
        apidOpts.Kd_init = Kstar(3);
    end

    OUT = run_apid_bandlock_smooth(T4_des, apidOpts);
end


function row = empty_metric_row()
    row = struct( ...
        'controller', "", ...
        'seed', NaN, ...
        'overshoot_pct', NaN, ...
        'rise_time_hr', NaN, ...
        'settling_time_hr', NaN, ...
        'time_in_30pct', NaN, ...
        'time_in_10pct', NaN, ...
        'IAE', NaN, ...
        'ISE', NaN, ...
        'RMSE', NaN, ...
        'control_energy', NaN);
end


function M = compute_metrics_from_out(OUT, controllerName, seed)
    M = empty_metric_row();

    M.controller = string(controllerName);
    M.seed = seed;

    [t, y] = get_measured_series(OUT);
    ref = extract_setpoint(OUT);

    e = ref-y;
    absError = abs(e);

    M.overshoot_pct = ...
        100*max(0, max(y)-ref)/ref;

    i10 = find(y >= 0.10*ref, 1, 'first');
    i90 = find(y >= 0.90*ref, 1, 'first');

    if ~isempty(i10) && ~isempty(i90) && i90 >= i10
        M.rise_time_hr = t(i90)-t(i10);
    end

    inside5 = absError <= 0.05*ref;
    M.settling_time_hr = first_permanent_entry(t, inside5);

    M.time_in_30pct = ...
        100*mean(absError <= 0.30*ref);

    M.time_in_10pct = ...
        100*mean(absError <= 0.10*ref);

    M.IAE = trapz(t, absError);
    M.ISE = trapz(t, e.^2);
    M.RMSE = sqrt(mean(e.^2));

    [tu, u] = get_ef_series(OUT);

    if numel(tu) >= 2
        M.control_energy = trapz(tu, u.^2);
    end
end


function tEntry = first_permanent_entry(t, inside)
    tEntry = NaN;

    for k = 1:numel(t)
        if all(inside(k:end))
            tEntry = t(k);
            return;
        end
    end
end


function summary = make_summary_table(F, A)
    Controller = [ ...
        "Optimized fixed PID"; ...
        "APID"];

    RMSE_mean = [ ...
        mean(F.RMSE, 'omitnan'); ...
        mean(A.RMSE, 'omitnan')];

    RMSE_std = [ ...
        std(F.RMSE, 0, 'omitnan'); ...
        std(A.RMSE, 0, 'omitnan')];

    IAE_mean = [ ...
        mean(F.IAE, 'omitnan'); ...
        mean(A.IAE, 'omitnan')];

    IAE_std = [ ...
        std(F.IAE, 0, 'omitnan'); ...
        std(A.IAE, 0, 'omitnan')];

    Overshoot_mean = [ ...
        mean(F.overshoot_pct, 'omitnan'); ...
        mean(A.overshoot_pct, 'omitnan')];

    Overshoot_std = [ ...
        std(F.overshoot_pct, 0, 'omitnan'); ...
        std(A.overshoot_pct, 0, 'omitnan')];

    TimeIn10_mean = [ ...
        mean(F.time_in_10pct, 'omitnan'); ...
        mean(A.time_in_10pct, 'omitnan')];

    TimeIn10_std = [ ...
        std(F.time_in_10pct, 0, 'omitnan'); ...
        std(A.time_in_10pct, 0, 'omitnan')];

    ControlEnergy_mean = [ ...
        mean(F.control_energy, 'omitnan'); ...
        mean(A.control_energy, 'omitnan')];

    ControlEnergy_std = [ ...
        std(F.control_energy, 0, 'omitnan'); ...
        std(A.control_energy, 0, 'omitnan')];

    summary = table( ...
        Controller, ...
        RMSE_mean, RMSE_std, ...
        IAE_mean, IAE_std, ...
        Overshoot_mean, Overshoot_std, ...
        TimeIn10_mean, TimeIn10_std, ...
        ControlEnergy_mean, ControlEnergy_std);
end


function C = compact_output(OUT, maxPoints)
    [tm, ym] = get_measured_series(OUT);
    [tt, yt] = get_true_series(OUT);
    [tu, u] = get_ef_series(OUT);
    [ta, amp] = get_amp_series(OUT);

    [C.t_meas_hr, C.y_meas] = ...
        downsample_curve(tm, ym, maxPoints);

    [C.t_true_hr, C.y_true] = ...
        downsample_curve(tt, yt, maxPoints);

    [C.t_ef_hr, C.ef] = ...
        downsample_curve(tu, u, maxPoints);

    [C.t_amp_hr, C.amp] = ...
        downsample_curve(ta, amp, maxPoints);
end


function make_trajectory_figure(F, A, T4_des, Kstar, seed, opts)
    visibility = on_off(opts.figure_visible);

    % Large canvas + loose tiled layout improve readability after IEEE scaling.
    fig = figure( ...
        'Color', 'w', ...
        'Visible', visibility, ...
        'Units', 'pixels', ...
        'Position', [80 80 2400 1650]);

    tl = tiledlayout(fig, 2, 2, ...
        'TileSpacing', 'loose', ...
        'Padding', 'loose');

    % Common plotting style.
    LW       = 3.0;
    LW_ref   = 2.6;
    MS       = 8;
    FS_axes   = 30;
    FS_label  = 30;
    FS_title  = 30;
    FS_legend = 28;
    FS_panel  = 30;

    % ==========================================================
    % A. Measured tracking
    % ==========================================================
    ax1 = nexttile(tl, 1);
    hold(ax1, 'on');

    hFixedA = plot(ax1, F.t_meas_hr, F.y_meas, ...
        '-o', ...
        'LineWidth', LW, ...
        'MarkerSize', MS, ...
        'MarkerIndices', marker_indices(numel(F.t_meas_hr), 18), ...
        'DisplayName', 'Offline-tuned fixed PID');

    hApidA = plot(ax1, A.t_meas_hr, A.y_meas, ...
        '-s', ...
        'LineWidth', LW, ...
        'MarkerSize', MS, ...
        'MarkerIndices', marker_indices(numel(A.t_meas_hr), 18), ...
        'DisplayName', 'APID');

    hSetA = yline(ax1, T4_des, '--k', ...
        'LineWidth', LW_ref, ...
        'DisplayName', 'Setpoint');

    xlabel(ax1, 'Time (h)');
    ylabel(ax1, 'Measured T_4^{ext} (a.u.)');
    title(ax1, 'Measured tracking');
    grid(ax1, 'on');

    add_y_headroom(ax1, [F.y_meas(:); A.y_meas(:); T4_des], 0.15);

    % ==========================================================
% B. True output
% ==========================================================
ax2 = nexttile(tl, 2);
hold(ax2, 'on');

plot(ax2, F.t_true_hr, F.y_true, ...
    '-', ...
    'LineWidth', LW, ...
    'DisplayName', 'Offline-tuned fixed PID');

plot(ax2, A.t_true_hr, A.y_true, ...
    '-', ...
    'LineWidth', LW, ...
    'DisplayName', 'APID');

yline(ax2, T4_des, '--k', ...
    'LineWidth', LW_ref, ...
    'DisplayName', 'Setpoint');

xlabel(ax2, 'Time (h)');
ylabel(ax2, 'True T_4^{ext} (a.u.)');
title(ax2, 'True output');
grid(ax2, 'on');

add_y_headroom(ax2, [F.y_true(:); A.y_true(:); T4_des], 0.15);

% ==========================================================
% C. EF command
% ==========================================================
ax3 = nexttile(tl, 3);
hold(ax3, 'on');

stairs(ax3, F.t_ef_hr, F.ef, ...
    '-', ...
    'LineWidth', LW, ...
    'DisplayName', 'Offline-tuned fixed PID');

stairs(ax3, A.t_ef_hr, A.ef, ...
    '-', ...
    'LineWidth', LW, ...
    'DisplayName', 'APID');

xlabel(ax3, 'Time (h)');
ylabel(ax3, 'EF command (a.u.)');
title(ax3, 'EF command');
grid(ax3, 'on');

add_y_headroom(ax3, [F.ef(:); A.ef(:)], 0.12);

% ==========================================================
% D. Applied amplitude
% ==========================================================
ax4 = nexttile(tl, 4);
hold(ax4, 'on');

stairs(ax4, F.t_amp_hr, F.amp, ...
    '-', ...
    'LineWidth', LW, ...
    'DisplayName', 'Offline-tuned fixed PID');

stairs(ax4, A.t_amp_hr, A.amp, ...
    '-', ...
    'LineWidth', LW, ...
    'DisplayName', 'APID');

xlabel(ax4, 'Time (h)');
ylabel(ax4, 'Applied amplitude A (a.u.)');
title(ax4, 'Applied amplitude');
grid(ax4, 'on');

add_y_headroom(ax4, [F.amp(:); A.amp(:)], 0.12);

    % ==========================================================
    % Consistent axes formatting
    % ==========================================================
    axesList = [ax1 ax2 ax3 ax4];

    for ii = 1:numel(axesList)
        ax = axesList(ii);

        ax.FontSize  = FS_axes;
        ax.LineWidth = 1.8;
        ax.Box       = 'on';
        ax.TickDir   = 'out';

        ax.XLabel.FontSize = FS_label;
        ax.YLabel.FontSize = FS_label;
        ax.Title.FontSize  = FS_title;
        ax.Title.FontWeight = 'bold';
    end

    % One common legend avoids four small legend boxes inside the panels.
    lgd = legend(ax1, [hFixedA hApidA hSetA], ...
        {'Offline-tuned fixed PID', 'APID', 'Setpoint'}, ...
        'Orientation', 'horizontal', ...
        'FontSize', FS_legend, ...
        'Box', 'on');
    lgd.Layout.Tile = 'south';

    % External A--D panel labels. Use figure annotations rather than
    % text(ax,...) so the labels are not clipped by the axes.
    drawnow;
    add_panel_label_outside(fig, ax1, 'A', FS_panel);
    add_panel_label_outside(fig, ax2, 'B', FS_panel);
    add_panel_label_outside(fig, ax3, 'C', FS_panel);
    add_panel_label_outside(fig, ax4, 'D', FS_panel);
    drawnow;

    if opts.figure_visible
        fprintf(['Trajectory figure: seed %d, ' ...
                 'K*=[%.6g, %.6g, %.6g]\n'], ...
                 seed, Kstar(1), Kstar(2), Kstar(3));
    end

    save_figure(fig, ...
        fullfile(opts.output_dir, ...
        'optimized_fixed_vs_apid_trajectories'), ...
        opts);

    close(fig);
end

function add_y_headroom(ax, values, fracTop)
    values = values(isfinite(values));
    if isempty(values)
        return;
    end

    ymin = min(0, min(values));
    ymax = max(values);
    span = max(ymax - ymin, max(abs(ymax), 1) * 0.05);

    ylim(ax, [ymin - 0.03*span, ymax + fracTop*span]);
end

function idx = marker_indices(n, maxMarkers)
    if n <= 0
        idx = [];
        return;
    end

    if n <= maxMarkers
        idx = 1:n;
    else
        idx = unique(round(linspace(1, n, maxMarkers)));
    end
end

function h = add_panel_label_outside(fig, ax, labelText, FS_panel)
    drawnow;

    oldUnits = ax.Units;
    ax.Units = 'normalized';
    pos = ax.Position;
    ax.Units = oldUnits;

    % Place the label just above and left of the axes, but keep it safely
    % inside the figure canvas so vector export cannot crop it.
    x = max(0.010, pos(1) - 0.035);
    y = min(0.965, pos(2) + pos(4) + 0.008);

    h = annotation(fig, 'textbox', ...
        [x y 0.04 0.04], ...
        'String', labelText, ...
        'FontSize', FS_panel, ...
        'FontWeight', 'bold', ...
        'HorizontalAlignment', 'left', ...
        'VerticalAlignment', 'bottom', ...
        'EdgeColor', 'none', ...
        'BackgroundColor', 'none', ...
        'Margin', 0, ...
        'FitBoxToText', 'on');
end

function make_metric_bar_figure(F, A, opts)
    labels = categorical({'Optimized fixed PID', 'APID'});
    labels = reordercats(labels, ...
        {'Optimized fixed PID', 'APID'});

    fig = figure( ...
        'Color', 'w', ...
        'Visible', on_off(opts.figure_visible), ...
        'Position', [50 50 1800 900]);

    tiledlayout(fig, 2, 3, ...
        'TileSpacing', 'compact', ...
        'Padding', 'compact');

    bar_metric(nexttile, labels, ...
        [mean(F.RMSE,'omitnan'), mean(A.RMSE,'omitnan')], ...
        [std(F.RMSE,0,'omitnan'), std(A.RMSE,0,'omitnan')], ...
        'RMSE', 'Tracking RMSE');

    bar_metric(nexttile, labels, ...
        [mean(F.IAE,'omitnan'), mean(A.IAE,'omitnan')], ...
        [std(F.IAE,0,'omitnan'), std(A.IAE,0,'omitnan')], ...
        'IAE (a.u. h)', 'Integral absolute error');

    bar_metric(nexttile, labels, ...
        [mean(F.overshoot_pct,'omitnan'), ...
         mean(A.overshoot_pct,'omitnan')], ...
        [std(F.overshoot_pct,0,'omitnan'), ...
         std(A.overshoot_pct,0,'omitnan')], ...
        'Overshoot (%)', 'Peak overshoot');

    bar_metric(nexttile, labels, ...
        [mean(F.time_in_10pct,'omitnan'), ...
         mean(A.time_in_10pct,'omitnan')], ...
        [std(F.time_in_10pct,0,'omitnan'), ...
         std(A.time_in_10pct,0,'omitnan')], ...
        'Time in \pm10% band (%)', 'Target-band retention');

    bar_metric(nexttile, labels, ...
        [mean(F.control_energy,'omitnan'), ...
         mean(A.control_energy,'omitnan')], ...
        [std(F.control_energy,0,'omitnan'), ...
         std(A.control_energy,0,'omitnan')], ...
        'EF energy (a.u.^2 h)', 'Control effort');

    bar_metric(nexttile, labels, ...
        [mean(F.rise_time_hr,'omitnan'), ...
         mean(A.rise_time_hr,'omitnan')], ...
        [std(F.rise_time_hr,0,'omitnan'), ...
         std(A.rise_time_hr,0,'omitnan')], ...
        'Rise time (h)', 'Rise time');

    format_figure(fig);

    save_figure(fig, fullfile(opts.output_dir, ...
        'optimized_fixed_vs_apid_metric_bars'), opts);

    close(fig);
end


function bar_metric(ax, labels, means, sds, yLabel, ttl)
    hold(ax, 'on');

    bar(ax, labels, means, ...
        'LineWidth', 1.5);

    errorbar(ax, 1:2, means, sds, ...
        'k.', 'LineWidth', 2.2, 'CapSize', 14);

    ylabel(ax, yLabel);
    title(ax, ttl);
    grid(ax, 'on');
end


function format_figure(fig)

    axesHandles = findall(fig, 'Type', 'axes');

    for i = 1:numel(axesHandles)
        ax = axesHandles(i);

        ax.FontSize = 26;
        ax.LineWidth = 2.0;
        ax.Box = 'on';
        ax.TickDir = 'out';

        ax.XLabel.FontSize = 26;
        ax.YLabel.FontSize = 26;

        ax.Title.FontSize = 28;
        ax.Title.FontWeight = 'bold';
    end

    legends = findall(fig, 'Type', 'legend');

    for i = 1:numel(legends)
        legends(i).FontSize = 26;
        legends(i).Box = 'on';
    end

    drawnow;
end

function save_figure(fig, baseName, opts)
    if opts.save_pdf
        exportgraphics(fig, [baseName '.pdf'], ...
            'ContentType', 'vector', ...
            'BackgroundColor', 'white');
    end

    if opts.save_png
        exportgraphics(fig, [baseName '.png'], ...
            'Resolution', opts.png_resolution, ...
            'BackgroundColor', 'white');
    end
end


function value = on_off(flag)
    if flag
        value = 'on';
    else
        value = 'off';
    end
end


function [xp, yp] = downsample_curve(x, y, maxPoints)
    x = x(:);
    y = y(:);

    n = min(numel(x), numel(y));
    x = x(1:n);
    y = y(1:n);

    valid = isfinite(x) & isfinite(y);
    x = x(valid);
    y = y(valid);

    if numel(x) <= maxPoints
        xp = x;
        yp = y;
        return;
    end

    idx = unique(round(linspace(1, numel(x), maxPoints)));
    xp = x(idx);
    yp = y(idx);
end


 
function add_project_paths(projectRoot)
%ADD_PROJECT_PATHS Add all required project folders.

    folders = { ...
        projectRoot, ...
        fullfile(projectRoot, 'common'), ...
        fullfile(projectRoot, 'common', 'plant'), ...
        fullfile(projectRoot, 'common', 'timing'), ...
        fullfile(projectRoot, 'common', 'utils'), ...
        fullfile(projectRoot, 'apid'), ...
        fullfile(projectRoot, 'apid', 'src'), ...
        fullfile(projectRoot, 'experiments')};

    for i = 1:numel(folders)
        if exist(folders{i}, 'dir')
            addpath(folders{i});
        end
    end
end

function tag = numeric_file_tag(value)
    tag = sprintf('%.6g', value);
    tag = strrep(tag, '-', 'm');
    tag = strrep(tag, '.', 'p');
end


function [t, y] = get_measured_series(OUT)
    if isfield(OUT, 'T4_meas')
        y = OUT.T4_meas(:);
    elseif isfield(OUT, 'T4_meas_used')
        y = OUT.T4_meas_used(:);
    elseif isfield(OUT, 'T4_meas_raw')
        y = OUT.T4_meas_raw(:);
    else
        error('Could not find measured T4 output.');
    end

    tRaw = OUT.t_meas(:);
    n = min(numel(tRaw), numel(y));
    tRaw = tRaw(1:n);
    y = y(1:n);

    if isfield(OUT, 'time_hr')
        if max(tRaw) > 1.5*max(OUT.time_hr(:))
            t = tRaw/60;
        else
            t = tRaw;
        end
    elseif max(tRaw) > 200
        t = tRaw/60;
    else
        t = tRaw;
    end

    valid = isfinite(t) & isfinite(y);
    t = t(valid);
    y = y(valid);
end


function [t, y] = get_true_series(OUT)
    t = get_time_hr(OUT);

    if isfield(OUT, 'T4_true')
        y = OUT.T4_true(:);
    elseif isfield(OUT, 'T4_log')
        y = OUT.T4_log(:);
    elseif isfield(OUT, 'y') && size(OUT.y,2) >= 4
        y = OUT.y(:,4);
    else
        error('Could not find true T4 output.');
    end

    n = min(numel(t), numel(y));
    t = t(1:n);
    y = y(1:n);

    valid = isfinite(t) & isfinite(y);
    t = t(valid);
    y = y(valid);
end


function [t, u] = get_ef_series(OUT)
    if isfield(OUT, 'u_pid')
        u = OUT.u_pid(:);
        t = get_time_hr(OUT);
    elseif isfield(OUT, 'EF_cmd')
        u = OUT.EF_cmd(:);
        Ts = extract_Ts(OUT);
        t = (0:numel(u)-1)'*Ts/60;
    elseif isfield(OUT, 'u_cmd')
        u = OUT.u_cmd(:);
        Ts = extract_Ts(OUT);
        t = (0:numel(u)-1)'*Ts/60;
    else
        error('Could not find EF command.');
    end

    n = min(numel(t), numel(u));
    t = t(1:n);
    u = u(1:n);

    valid = isfinite(t) & isfinite(u);
    t = t(valid);
    u = u(valid);
end


function [t, A] = get_amp_series(OUT)
    if isfield(OUT, 'A_t')
        A = OUT.A_t(:);
        t = get_time_hr(OUT);
    elseif isfield(OUT, 'A_cmd')
        A = OUT.A_cmd(:);
        Ts = extract_Ts(OUT);
        t = (0:numel(A)-1)'*Ts/60;
    elseif isfield(OUT, 'A_hist')
        Aall = OUT.A_hist(:);
        validA = isfinite(Aall);

        if isfield(OUT, 'sample_times')
            sampleTimes = OUT.sample_times(:);
            n = min(numel(sampleTimes), numel(Aall));
            sampleTimes = sampleTimes(1:n);
            Aall = Aall(1:n);
            validA = validA(1:n);
            t = sampleTimes(validA)/60;
            A = Aall(validA);
        else
            A = Aall(validA);
            Ts = extract_Ts(OUT);
            t = (0:numel(A)-1)'*Ts/60;
        end
    else
        error('Could not find amplitude output.');
    end

    n = min(numel(t), numel(A));
    t = t(1:n);
    A = A(1:n);

    valid = isfinite(t) & isfinite(A);
    t = t(valid);
    A = A(valid);
end


function t = get_time_hr(OUT)
    if isfield(OUT, 'time_hr')
        t = OUT.time_hr(:);
    elseif isfield(OUT, 'time')
        t = OUT.time(:)/60;
    else
        error('Could not find time vector.');
    end
end


function ref = extract_setpoint(OUT)
    if isfield(OUT, 'setpoint')
        ref = OUT.setpoint;
    elseif isfield(OUT, 'T4_des')
        ref = OUT.T4_des;
    elseif isfield(OUT, 'params') && ...
            isstruct(OUT.params) && ...
            isfield(OUT.params, 'T4_des')
        ref = OUT.params.T4_des;
    elseif isfield(OUT, 'P') && ...
            isstruct(OUT.P) && ...
            isfield(OUT.P, 'T4_des')
        ref = OUT.P.T4_des;
    else
        error('Could not find setpoint.');
    end
end


function Ts = extract_Ts(OUT)
    if isfield(OUT, 'P') && ...
            isstruct(OUT.P) && ...
            isfield(OUT.P, 'Ts_min')
        Ts = OUT.P.Ts_min;
    elseif isfield(OUT, 'params') && ...
            isstruct(OUT.params) && ...
            isfield(OUT.params, 'Ts_min')
        Ts = OUT.params.Ts_min;
    elseif isfield(OUT, 'Ts_min')
        Ts = OUT.Ts_min;
    elseif isfield(OUT, 'sample_times') && ...
            numel(OUT.sample_times) >= 2
        Ts = median(diff(OUT.sample_times));
    else
        Ts = 120;
    end
end
