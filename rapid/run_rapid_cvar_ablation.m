function RESULTS = run_rapid_cvar_ablation(T4_des, opts)
%RUN_RAPID_CVAR_ABLATION_SAFE Memory-safe paired RAPID CVaR ablation.
%
%   RESULTS = RUN_RAPID_CVAR_ABLATION_SAFE()
%   RESULTS = RUN_RAPID_CVAR_ABLATION_SAFE(T4_des)
%   RESULTS = RUN_RAPID_CVAR_ABLATION_SAFE(T4_des, opts)
%
% This driver compares
%
%   lambda_cvar = 0.6: original mean--CVaR RAPID objective,
%   lambda_cvar = 0.0: mean scenario cost only.
%
% For every random seed, both cases use the same random-stream setup.
% Only opts.robust_lambda_cvar changes.
%
% The code is designed to avoid the failure mode of the earlier driver:
%   1. each completed case is checkpointed immediately;
%   2. only compact/downsampled trajectories are retained in memory;
%   3. the target band is drawn with a four-vertex rectangle;
%   4. the final MAT file contains compact outputs rather than ten
%      full-resolution trajectories;
%   5. interrupted runs can resume from compatible checkpoints.
%
% Required existing files:
%   run_rapid_clean5.m
%   src/rapid_*.m
%
% Important options:
%   opts.seeds                  default 1:5
%   opts.lambda_values          default [0.6 0]
%   opts.output_dir             output folder
%   opts.resume                 reuse compatible checkpoints, default true
%   opts.max_plot_points        default 6000
%   opts.existing_summary_file  optional five-seed summary CSV used only
%                               for the metric figure
%
% Example: complete paper experiment
%   opts = struct();
%   opts.seeds = 1:5;
%   opts.output_dir = fullfile(pwd,'results','cvar_ablation_safe');
%   RESULTS = run_rapid_cvar_ablation_safe(25, opts);
%
% Example: recover figures using the already completed five-seed summary
% while rerunning only paired seed 1 for trajectories
%   opts = struct();
%   opts.seeds = 1;
%   opts.output_dir = fullfile(pwd,'results','cvar_ablation_recovery');
%   opts.existing_summary_file = fullfile(pwd,'results','cvar_ablation', ...
%       'rapid_cvar_ablation_summary.csv');
%   RESULTS = run_rapid_cvar_ablation_safe(25, opts);
%
% Quick smoke test
%   opts = struct();
%   opts.seeds = 1;
%   opts.sim_hours = 2;
%   opts.output_dir = fullfile(pwd,'cvar_ablation_test');
%   RESULTS = run_rapid_cvar_ablation_safe(25, opts);

    if nargin < 1 || isempty(T4_des)
        T4_des = 25;
    end
    if nargin < 2 || isempty(opts)
        opts = struct();
    end

    repoRoot = fileparts(mfilename('fullpath'));
    addpath(repoRoot);

    srcPath = fullfile(repoRoot, 'src');
    if exist(srcPath, 'dir')
        addpath(srcPath);
    end

    opts = apply_ablation_defaults(opts, repoRoot);
    validate_ablation_options(opts);

    if ~exist(opts.output_dir, 'dir')
        mkdir(opts.output_dir);
    end

    checkpointDir = fullfile(opts.output_dir, 'checkpoints');
    if ~exist(checkpointDir, 'dir')
        mkdir(checkpointDir);
    end

    lambda_values = opts.lambda_values(:)';
    seeds = opts.seeds(:)';
    nLambda = numel(lambda_values);
    nSeeds = numel(seeds);

    % OUTS contains only compact outputs, not full-resolution trajectories.
    OUTS = cell(nSeeds, nLambda);
    metricRows = repmat(empty_metric_row(), nSeeds*nLambda, 1);
    row = 0;

    fprintf('\nRAPID CVaR ablation (safe driver)\n');
    fprintf('Setpoint: %.6g\n', T4_des);
    fprintf('Seeds: %s\n', mat2str(seeds));
    fprintf('Lambda values: %s\n', mat2str(lambda_values));
    fprintf('M = %d, q = %.3f\n', opts.robust_M, opts.robust_cvar_q);
    fprintf('Resume = %d\n\n', opts.resume);

    for iseed = 1:nSeeds
        seed = seeds(iseed);

        for ilam = 1:nLambda
            lambda_cvar = lambda_values(ilam);
            row = row + 1;

            run_opts = build_run_options(opts, seed, lambda_cvar);
            caseMeta = build_case_metadata( ...
                T4_des, seed, lambda_cvar, run_opts);

            checkpointFile = fullfile(checkpointDir, ...
                sprintf('rapid_seed_%d_lambda_%s.mat', ...
                seed, lambda_file_tag(lambda_cvar)));

            reused = false;
            if opts.resume && exist(checkpointFile, 'file')
                loaded = load(checkpointFile, ...
                    'compactOUT', 'metricRow', 'caseMeta');

                if isfield(loaded, 'compactOUT') && ...
                        isfield(loaded, 'metricRow') && ...
                        isfield(loaded, 'caseMeta') && ...
                        isequaln(loaded.caseMeta, caseMeta)

                    compactOUT = loaded.compactOUT;
                    metricRow = loaded.metricRow;
                    reused = true;

                    fprintf(['Reusing checkpoint: seed %d, ' ...
                        'lambda = %.3g\n'], seed, lambda_cvar);
                end
            end

            if ~reused
                fprintf('Running seed %d, lambda = %.3g ...\n', ...
                    seed, lambda_cvar);
                caseTimer = tic;

                OUT = run_rapid_clean5(T4_des, run_opts);

                metricRow = compute_metrics( ...
                    OUT, lambda_cvar, opts.band_fraction, ...
                    opts.settling_dwell_hours);

                compactOUT = compact_output(OUT, opts.max_plot_points);

                % Save the compact trajectory and metrics immediately.
                save(checkpointFile, ...
                    'compactOUT', 'metricRow', 'caseMeta', 'run_opts', ...
                    '-v7');

                fprintf(['Completed and checkpointed seed %d, ' ...
                    'lambda = %.3g in %.2f min.\n'], ...
                    seed, lambda_cvar, toc(caseTimer)/60);

                clear OUT
            end

            OUTS{iseed, ilam} = compactOUT;
            metricRows(row) = metricRow;
        end
    end

    perSeed = struct2table(metricRows);
    perSeed = sortrows(perSeed, ...
        {'seed','lambda_cvar'}, {'ascend','descend'});

    summary = build_summary_table( ...
        perSeed, lambda_values, opts.robust_cvar_q);
    paired = build_paired_difference_table(perSeed);

    perSeedFile = fullfile(opts.output_dir, ...
        'rapid_cvar_ablation_per_seed.csv');
    summaryFile = fullfile(opts.output_dir, ...
        'rapid_cvar_ablation_summary.csv');
    pairedFile = fullfile(opts.output_dir, ...
        'rapid_cvar_ablation_paired_differences.csv');

    writetable(perSeed, perSeedFile);
    writetable(summary, summaryFile);
    writetable(paired, pairedFile);

    latexFile = fullfile(opts.output_dir, ...
        'rapid_cvar_ablation_summary_table.tex');
    write_latex_table(summary, latexFile, opts.robust_cvar_q);

    % Use an existing five-seed summary for the metric plot when supplied.
    summaryForFigure = summary;
    if ~isempty(opts.existing_summary_file)
        if exist(opts.existing_summary_file, 'file')
            summaryForFigure = readtable(opts.existing_summary_file);
            validate_summary_columns(summaryForFigure);
            fprintf('Metric figure uses existing summary:\n%s\n', ...
                opts.existing_summary_file);
        else
            warning('Existing summary file not found: %s', ...
                opts.existing_summary_file);
        end
    end

    make_trajectory_figure( ...
        OUTS, seeds, lambda_values, T4_des, opts);
    make_metric_figure( ...
        summaryForFigure, summaryForFigure.Lambda(:)', opts);

    RESULTS = struct();
    RESULTS.T4_des = T4_des;
    RESULTS.options = opts;
    RESULTS.seeds = seeds;
    RESULTS.lambda_values = lambda_values;
    RESULTS.compact_outputs = OUTS;
    RESULTS.per_seed = perSeed;
    RESULTS.summary = summary;
    RESULTS.summary_used_for_metric_figure = summaryForFigure;
    RESULTS.paired_differences = paired;

    save(fullfile(opts.output_dir, ...
        'rapid_cvar_ablation_results_compact.mat'), ...
        'RESULTS', '-v7');

    fprintf('\nCompleted. Results written to:\n%s\n', opts.output_dir);
    fprintf('\nCompact summary from the cases run by this call:\n');
    disp(summary);
end


function run_opts = build_run_options(opts, seed, lambda_cvar)
    run_opts = struct();

    run_opts.dt_seconds = opts.dt_seconds;
    run_opts.sim_hours = opts.sim_hours;
    run_opts.do_plots = false;
    run_opts.verbose = opts.controller_verbose;
    run_opts.do_save_pdf = false;
    run_opts.output_dir = opts.output_dir;
    run_opts.rng_seed = seed;

    % Only lambda_cvar changes within each paired seed.
    run_opts.robust_M = opts.robust_M;
    run_opts.robust_cvar_q = opts.robust_cvar_q;
    run_opts.robust_lambda_cvar = lambda_cvar;

    % Same uncertainty configuration as the RAPID paper experiment.
    run_opts.param_scale_default = opts.param_scale_default;
    run_opts.kappa_scale = opts.kappa_scale;
    run_opts.K2_scale = opts.K2_scale;
    run_opts.d4_bias = opts.d4_bias;
    run_opts.d4_amp = opts.d4_amp;
    run_opts.act_gain_scale = opts.act_gain_scale;
    run_opts.delay_seconds = opts.delay_seconds;
    run_opts.jitter_seconds = opts.jitter_seconds;
end


function caseMeta = build_case_metadata(T4_des, seed, lambda_cvar, run_opts)
% Metadata used to reject stale or incompatible checkpoints.

    caseMeta = struct();
    caseMeta.T4_des = T4_des;
    caseMeta.seed = seed;
    caseMeta.lambda_cvar = lambda_cvar;
    caseMeta.dt_seconds = run_opts.dt_seconds;
    caseMeta.sim_hours = run_opts.sim_hours;
    caseMeta.robust_M = run_opts.robust_M;
    caseMeta.robust_cvar_q = run_opts.robust_cvar_q;
    caseMeta.param_scale_default = run_opts.param_scale_default;
    caseMeta.kappa_scale = run_opts.kappa_scale;
    caseMeta.K2_scale = run_opts.K2_scale;
    caseMeta.d4_bias = run_opts.d4_bias;
    caseMeta.d4_amp = run_opts.d4_amp;
    caseMeta.act_gain_scale = run_opts.act_gain_scale;
    caseMeta.delay_seconds = run_opts.delay_seconds;
    caseMeta.jitter_seconds = run_opts.jitter_seconds;
end


function tag = lambda_file_tag(lambda_cvar)
    tag = sprintf('%.6g', lambda_cvar);
    tag = strrep(tag, '-', 'm');
    tag = strrep(tag, '.', 'p');
end


function opts = apply_ablation_defaults(opts, repoRoot)
    opts = set_default(opts, 'seeds', 1:5);
    opts = set_default(opts, 'lambda_values', [0.6, 0.0]);
    opts = set_default(opts, 'robust_M', 5);
    opts = set_default(opts, 'robust_cvar_q', 0.80);
    opts = set_default(opts, 'dt_seconds', 0.1);
    opts = set_default(opts, 'sim_hours', 80);
    opts = set_default(opts, 'band_fraction', 0.05);
    opts = set_default(opts, 'settling_dwell_hours', 2);
    opts = set_default(opts, 'controller_verbose', false);
    opts = set_default(opts, 'figure_visible', false);
    opts = set_default(opts, 'save_png', true);
    opts = set_default(opts, 'save_pdf', true);
    opts = set_default(opts, 'resume', true);
    opts = set_default(opts, 'max_plot_points', 6000);
    opts = set_default(opts, 'existing_summary_file', '');
    opts = set_default(opts, 'output_dir', ...
        fullfile(repoRoot, 'results', 'cvar_ablation_safe'));

    % Same uncertainty defaults as rapid_apply_default_options.m.
    opts = set_default(opts, 'param_scale_default', 0.08);
    opts = set_default(opts, 'kappa_scale', 0.10);
    opts = set_default(opts, 'K2_scale', 0.10);
    opts = set_default(opts, 'd4_bias', 0.01);
    opts = set_default(opts, 'd4_amp', 0.01);
    opts = set_default(opts, 'act_gain_scale', 0.10);
    opts = set_default(opts, 'delay_seconds', 5);
    opts = set_default(opts, 'jitter_seconds', 2);
end


function opts = set_default(opts, fieldName, value)
    if ~isfield(opts, fieldName) || isempty(opts.(fieldName))
        opts.(fieldName) = value;
    end
end


function validate_ablation_options(opts)
    if isempty(opts.seeds) || any(~isfinite(opts.seeds)) || ...
            any(opts.seeds < 0) || any(mod(opts.seeds,1) ~= 0)
        error('opts.seeds must contain nonnegative integer seeds.');
    end

    if isempty(opts.lambda_values) || ...
            any(opts.lambda_values < 0) || ...
            any(opts.lambda_values > 1)
        error('opts.lambda_values must lie in [0,1].');
    end

    if opts.robust_M < 1 || mod(opts.robust_M,1) ~= 0
        error('opts.robust_M must be a positive integer.');
    end

    if opts.robust_cvar_q < 0 || opts.robust_cvar_q >= 1
        error('opts.robust_cvar_q must lie in [0,1).');
    end

    if opts.dt_seconds <= 0 || opts.sim_hours <= 0
        error('Time step and simulation duration must be positive.');
    end

    if opts.band_fraction <= 0 || opts.band_fraction >= 1
        error('opts.band_fraction must lie in (0,1).');
    end

    if opts.settling_dwell_hours < 0
        error('opts.settling_dwell_hours must be nonnegative.');
    end

    if opts.max_plot_points < 100 || ...
            mod(opts.max_plot_points,1) ~= 0
        error('opts.max_plot_points must be an integer of at least 100.');
    end
end


function validate_summary_columns(summary)
    required = { ...
        'Lambda', 'RMSE_mean', 'RMSE_std', ...
        'ISE_mean', 'ISE_std', 'ISE_CVaR', 'ISE_worst', ...
        'TimeInBand_mean', 'TimeInBand_std'};

    missing = required(~ismember(required, summary.Properties.VariableNames));
    if ~isempty(missing)
        error('Existing summary CSV is missing columns: %s', ...
            strjoin(missing, ', '));
    end
end


function compactOUT = compact_output(OUT, maxPoints)
% Retain all window-level values and downsample only fine-grid signals.

    compactOUT = struct();

    fineFields = { ...
        'time_hr', ...
        'T4_log', ...
        'u_pid', ...
        'EF_supplied_log', ...
        'A_t'};

    lengths = nan(size(fineFields));
    for i = 1:numel(fineFields)
        lengths(i) = numel(OUT.(fineFields{i}));
    end
    nFine = min(lengths);

    if nFine < 1
        error('Cannot compact an empty RAPID output.');
    end

    if nFine <= maxPoints
        idx = 1:nFine;
    else
        idx = unique(round(linspace(1, nFine, maxPoints)));
    end

    for i = 1:numel(fineFields)
        value = OUT.(fineFields{i});
        value = value(:);
        compactOUT.(fineFields{i}) = value(idx);
    end

    compactOUT.T4_des = OUT.T4_des;
    compactOUT.masterSeed = OUT.masterSeed;

    % Window-level quantities are already small and are kept unchanged.
    windowFields = { ...
        'sample_times', ...
        'A_hist', ...
        'Kp_hist', ...
        'Ki_hist', ...
        'Kd_hist', ...
        'cost_hist', ...
        't_meas', ...
        'T4_meas_raw', ...
        'T4_meas_used', ...
        'bias_hat_hist', ...
        'lock_hist'};

    for i = 1:numel(windowFields)
        compactOUT.(windowFields{i}) = OUT.(windowFields{i});
    end
end


function row = empty_metric_row()
    row = struct( ...
        'seed', NaN, ...
        'lambda_cvar', NaN, ...
        'robust_M', NaN, ...
        'cvar_q', NaN, ...
        'setpoint', NaN, ...
        'rmse', NaN, ...
        'mae', NaN, ...
        'iae_h', NaN, ...
        'ise_h', NaN, ...
        'peak_overshoot', NaN, ...
        'peak_overshoot_pct', NaN, ...
        'final_abs_error', NaN, ...
        'time_in_band_pct', NaN, ...
        'practical_settling_h', NaN, ...
        'first_band_lock_h', NaN, ...
        'band_lock_fraction_pct', NaN, ...
        'mean_amplitude', NaN, ...
        'max_amplitude', NaN, ...
        'amplitude_energy_h', NaN, ...
        'mean_abs_EF', NaN, ...
        'max_abs_EF', NaN, ...
        'EF_energy_h', NaN, ...
        'mean_abs_pid_command', NaN, ...
        'final_Kp', NaN, ...
        'final_Ki', NaN, ...
        'final_Kd', NaN, ...
        'mean_logged_objective', NaN, ...
        'max_logged_objective', NaN);
end


function row = compute_metrics(OUT, lambda_cvar, band_fraction, dwell_h)
    row = empty_metric_row();

    t = OUT.time_hr(:);
    y = OUT.T4_log(:);
    n = min(numel(t), numel(y));
    t = t(1:n);
    y = y(1:n);

    mask = isfinite(t) & isfinite(y);
    t = t(mask);
    y = y(mask);

    if numel(t) < 2
        error('The output trajectory contains fewer than two valid samples.');
    end

    r = OUT.T4_des;
    e = r-y;
    abs_e = abs(e);
    band_abs = band_fraction*abs(r);
    inside = abs_e <= band_abs;

    row.seed = OUT.masterSeed;
    row.lambda_cvar = lambda_cvar;
    row.robust_M = OUT.opts.robust_M;
    row.cvar_q = OUT.opts.robust_cvar_q;
    row.setpoint = r;

    row.rmse = sqrt(mean(e.^2));
    row.mae = mean(abs_e);
    row.iae_h = trapz(t, abs_e);
    row.ise_h = trapz(t, e.^2);
    row.peak_overshoot = max(0, max(y-r));

    if r ~= 0
        row.peak_overshoot_pct = ...
            100*row.peak_overshoot/abs(r);
    end

    row.final_abs_error = abs_e(end);
    row.time_in_band_pct = 100*mean(inside);
    row.practical_settling_h = ...
        first_dwell_entry_time(t, inside, dwell_h);

    [tA, A] = finite_pair(OUT.time_hr(:), OUT.A_t(:));
    if ~isempty(A)
        row.mean_amplitude = mean(A);
        row.max_amplitude = max(A);

        if numel(A) >= 2
            row.amplitude_energy_h = trapz(tA, A.^2);
        end
    end

    [tEF, EF] = finite_pair( ...
        OUT.time_hr(:), OUT.EF_supplied_log(:));
    if ~isempty(EF)
        row.mean_abs_EF = mean(abs(EF));
        row.max_abs_EF = max(abs(EF));

        if numel(EF) >= 2
            row.EF_energy_h = trapz(tEF, EF.^2);
        end
    end

    [~, uPID] = finite_pair( ...
        OUT.time_hr(:), OUT.u_pid(:));
    if ~isempty(uPID)
        row.mean_abs_pid_command = mean(abs(uPID));
    end

    tLock = OUT.sample_times(:)/60;
    lock = logical(OUT.lock_hist(:));
    nLock = min(numel(tLock), numel(lock));
    tLock = tLock(1:nLock);
    lock = lock(1:nLock);

    validLock = isfinite(tLock);
    tLock = tLock(validLock);
    lock = lock(validLock);

    if ~isempty(lock)
        row.band_lock_fraction_pct = 100*mean(lock);

        firstLock = find(lock, 1, 'first');
        if ~isempty(firstLock)
            row.first_band_lock_h = tLock(firstLock);
        end
    end

    row.final_Kp = last_finite(OUT.Kp_hist);
    row.final_Ki = last_finite(OUT.Ki_hist);
    row.final_Kd = last_finite(OUT.Kd_hist);

    J = OUT.cost_hist(:);
    J = J(isfinite(J));

    if ~isempty(J)
        row.mean_logged_objective = mean(J);
        row.max_logged_objective = max(J);
    end
end


function [x, y] = finite_pair(x, y)
    n = min(numel(x), numel(y));
    x = x(1:n);
    y = y(1:n);

    mask = isfinite(x) & isfinite(y);
    x = x(mask);
    y = y(mask);
end


function value = last_finite(x)
    x = x(:);
    idx = find(isfinite(x), 1, 'last');

    if isempty(idx)
        value = NaN;
    else
        value = x(idx);
    end
end


function tEntry = first_dwell_entry_time(t, inside, dwell_h)
% First start time of a continuous in-band interval lasting dwell_h.

    tEntry = NaN;

    if isempty(t)
        return;
    end

    if dwell_h <= 0
        idx = find(inside, 1, 'first');

        if ~isempty(idx)
            tEntry = t(idx);
        end
        return;
    end

    startIdx = [];

    for i = 1:numel(t)
        if inside(i)
            if isempty(startIdx)
                startIdx = i;
            end

            if t(i)-t(startIdx) >= dwell_h
                tEntry = t(startIdx);
                return;
            end
        else
            startIdx = [];
        end
    end
end


function summary = build_summary_table(perSeed, lambda_values, q)
    nLambda = numel(lambda_values);

    Lambda = nan(nLambda,1);
    Nseeds = nan(nLambda,1);

    RMSE_mean = nan(nLambda,1);
    RMSE_std = nan(nLambda,1);
    ISE_mean = nan(nLambda,1);
    ISE_std = nan(nLambda,1);
    ISE_CVaR = nan(nLambda,1);
    ISE_worst = nan(nLambda,1);

    Overshoot_mean = nan(nLambda,1);
    Overshoot_std = nan(nLambda,1);
    TimeInBand_mean = nan(nLambda,1);
    TimeInBand_std = nan(nLambda,1);
    Settling_mean_h = nan(nLambda,1);
    MaxAmplitude_mean = nan(nLambda,1);
    EFEnergy_mean_h = nan(nLambda,1);

    for i = 1:nLambda
        lam = lambda_values(i);
        T = perSeed(abs(perSeed.lambda_cvar-lam) < 1e-12, :);

        Lambda(i) = lam;
        Nseeds(i) = height(T);

        RMSE_mean(i) = mean_omitnan(T.rmse);
        RMSE_std(i) = std_omitnan(T.rmse);
        ISE_mean(i) = mean_omitnan(T.ise_h);
        ISE_std(i) = std_omitnan(T.ise_h);
        ISE_CVaR(i) = empirical_upper_cvar(T.ise_h, q);
        ISE_worst(i) = max_omitnan(T.ise_h);

        Overshoot_mean(i) = mean_omitnan(T.peak_overshoot);
        Overshoot_std(i) = std_omitnan(T.peak_overshoot);
        TimeInBand_mean(i) = mean_omitnan(T.time_in_band_pct);
        TimeInBand_std(i) = std_omitnan(T.time_in_band_pct);
        Settling_mean_h(i) = ...
            mean_omitnan(T.practical_settling_h);
        MaxAmplitude_mean(i) = ...
            mean_omitnan(T.max_amplitude);
        EFEnergy_mean_h(i) = ...
            mean_omitnan(T.EF_energy_h);
    end

    summary = table( ...
        Lambda, Nseeds, ...
        RMSE_mean, RMSE_std, ...
        ISE_mean, ISE_std, ISE_CVaR, ISE_worst, ...
        Overshoot_mean, Overshoot_std, ...
        TimeInBand_mean, TimeInBand_std, ...
        Settling_mean_h, MaxAmplitude_mean, EFEnergy_mean_h);
end


function paired = build_paired_difference_table(perSeed)
% Paired differences: lambda=0.6 minus lambda=0.

    T06 = perSeed(abs(perSeed.lambda_cvar-0.6) < 1e-12, :);
    T00 = perSeed(abs(perSeed.lambda_cvar-0.0) < 1e-12, :);

    commonSeeds = intersect(T06.seed, T00.seed);

    if isempty(commonSeeds)
        paired = table();
        return;
    end

    n = numel(commonSeeds);
    seed = commonSeeds(:);

    delta_RMSE = nan(n,1);
    delta_ISE = nan(n,1);
    delta_overshoot = nan(n,1);
    delta_time_in_band_pct = nan(n,1);
    delta_EF_energy_h = nan(n,1);
    delta_max_amplitude = nan(n,1);

    for i = 1:n
        a = T06(T06.seed == seed(i), :);
        b = T00(T00.seed == seed(i), :);

        delta_RMSE(i) = a.rmse(1)-b.rmse(1);
        delta_ISE(i) = a.ise_h(1)-b.ise_h(1);
        delta_overshoot(i) = ...
            a.peak_overshoot(1)-b.peak_overshoot(1);
        delta_time_in_band_pct(i) = ...
            a.time_in_band_pct(1)-b.time_in_band_pct(1);
        delta_EF_energy_h(i) = ...
            a.EF_energy_h(1)-b.EF_energy_h(1);
        delta_max_amplitude(i) = ...
            a.max_amplitude(1)-b.max_amplitude(1);
    end

    paired = table( ...
        seed, delta_RMSE, delta_ISE, delta_overshoot, ...
        delta_time_in_band_pct, delta_EF_energy_h, ...
        delta_max_amplitude);
end


function value = empirical_upper_cvar(x, q)
    x = x(:);
    x = x(isfinite(x));

    if isempty(x)
        value = NaN;
        return;
    end

    k = max(1, ceil((1-q)*numel(x)));
    x = sort(x, 'descend');
    value = mean(x(1:k));
end


function y = mean_omitnan(x)
    x = x(isfinite(x));

    if isempty(x)
        y = NaN;
    else
        y = mean(x);
    end
end


function y = std_omitnan(x)
    x = x(isfinite(x));

    if isempty(x)
        y = NaN;
    elseif isscalar(x)
        y = 0;
    else
        y = std(x, 0);
    end
end


function y = max_omitnan(x)
    x = x(isfinite(x));

    if isempty(x)
        y = NaN;
    else
        y = max(x);
    end
end


function make_trajectory_figure( ...
        OUTS, seeds, lambda_values, T4_des, opts)
% Plot a representative paired-seed comparison.

    if opts.figure_visible
        visibility = 'on';
    else
        visibility = 'off';
    end

    colors = lines(numel(lambda_values));

    seedIndex = 1;
    seed = seeds(seedIndex);

    fig = figure( ...
        'Color','w', ...
        'Visible',visibility, ...
        'Position',[100 100 1450 980]);

    tiledlayout(fig, 2, 2, ...
        'TileSpacing','compact', ...
        'Padding','compact');

    %==============================================================%
    % 1. Tracking
    %==============================================================%
    ax1 = nexttile;
    hold(ax1,'on');

    refOUT = OUTS{seedIndex,1};
    tStart = refOUT.time_hr(1);
    tEnd = refOUT.time_hr(end);

    bandLo = T4_des*(1-opts.band_fraction);
    bandHi = T4_des*(1+opts.band_fraction);

    bandLabel = sprintf('\\pm%.0f%% band', ...
        100*opts.band_fraction);

    % Four vertices only: avoids a multi-million-vertex patch.
    patch(ax1, ...
        [tStart, tEnd, tEnd, tStart], ...
        [bandLo, bandLo, bandHi, bandHi], ...
        [0.92 0.92 0.92], ...
        'EdgeColor','none', ...
        'FaceAlpha',0.5, ...
        'DisplayName',bandLabel);

    yline(ax1, T4_des, '--k', ...
        'LineWidth',1.5, ...
        'DisplayName','Setpoint');

    for j = 1:numel(lambda_values)
        OUT = OUTS{seedIndex,j};

        plot(ax1, OUT.time_hr(:), OUT.T4_log(:), ...
            'LineWidth',2, ...
            'Color',colors(j,:), ...
            'DisplayName',lambda_label(lambda_values(j)));
    end

    xlabel(ax1,'Time (h)');
    ylabel(ax1,'T_4^{ext} (a.u.)');
    title(ax1,sprintf('Tracking, paired seed %d',seed));
    xlim(ax1,[tStart,tEnd]);
    grid(ax1,'on');
    legend(ax1,'Location','best');

    %==============================================================%
    % 2. Per-window amplitude
    %==============================================================%
    ax2 = nexttile;
    hold(ax2,'on');

    for j = 1:numel(lambda_values)
        OUT = OUTS{seedIndex,j};

        sampleTimes = OUT.sample_times(:);
        A_hist = OUT.A_hist(:);

        n = min(numel(sampleTimes), numel(A_hist));
        sampleTimes = sampleTimes(1:n);
        A_hist = A_hist(1:n);

        mask = isfinite(sampleTimes) & isfinite(A_hist);

        stairs(ax2, sampleTimes(mask)/60, A_hist(mask), ...
            'LineWidth',2, ...
            'Color',colors(j,:), ...
            'DisplayName',lambda_label(lambda_values(j)));
    end

    xlabel(ax2,'Time (h)');
    ylabel(ax2,'Amplitude A (a.u.)');
    title(ax2,'Per-window EF amplitude');
    xlim(ax2,[tStart,tEnd]);
    grid(ax2,'on');
    legend(ax2,'Location','best');

    %==============================================================%
    % 3. Supplied EF
    %==============================================================%
    ax3 = nexttile;
    hold(ax3,'on');

    for j = 1:numel(lambda_values)
        OUT = OUTS{seedIndex,j};

        plot(ax3, OUT.time_hr(:), ...
            OUT.EF_supplied_log(:), ...
            'LineWidth',1.8, ...
            'Color',colors(j,:), ...
            'DisplayName',lambda_label(lambda_values(j)));
    end

    xlabel(ax3,'Time (h)');
    ylabel(ax3,'EF supplied (a.u.)');
    title(ax3,'Applied EF with delay and jitter');
    xlim(ax3,[tStart,tEnd]);
    grid(ax3,'on');
    legend(ax3,'Location','best');

    %==============================================================%
    % 4. Adaptive gains
    %==============================================================%
    ax4 = nexttile;
    hold(ax4,'on');

    lineStyles = {'-','--',':'};
    gainNames = {'K_p','K_i','K_d'};

    for j = 1:numel(lambda_values)
        OUT = OUTS{seedIndex,j};

        tK = OUT.sample_times(:)/60;
        gains = { ...
            OUT.Kp_hist(:), ...
            OUT.Ki_hist(:), ...
            OUT.Kd_hist(:)};

        for g = 1:3
            n = min(numel(tK), numel(gains{g}));
            tKg = tK(1:n);
            gainValues = gains{g}(1:n);

            mask = isfinite(tKg) & isfinite(gainValues);

            stairs(ax4, tKg(mask), gainValues(mask), ...
                'LineWidth',1.8, ...
                'LineStyle',lineStyles{g}, ...
                'Color',colors(j,:), ...
                'DisplayName',sprintf('%s, %s', ...
                    gainNames{g}, ...
                    lambda_label(lambda_values(j))));
        end
    end

    xlabel(ax4,'Time (h)');
    ylabel(ax4,'Adaptive PID gain');
    title(ax4,'Gain trajectories');
    xlim(ax4,[tStart,tEnd]);
    grid(ax4,'on');
    legend(ax4, ...
        'Location','best', ...
        'NumColumns',2);

    set(findall(fig,'Type','axes'), ...
        'FontSize',15, ...
        'LineWidth',1.1, ...
        'Box','on');

    drawnow;

    base = fullfile(opts.output_dir, ...
        'rapid_cvar_ablation_trajectories');
    save_figure_files(fig, base, opts);

    close(fig);
end


function make_metric_figure(summary, lambda_values, opts)
    if opts.figure_visible
        visibility = 'on';
    else
        visibility = 'off';
    end

    fig = figure( ...
        'Color','w', ...
        'Visible',visibility, ...
        'Position',[100 100 1300 900]);

    tiledlayout(fig, 2, 2, ...
        'TileSpacing','compact', ...
        'Padding','compact');

    labels = cell(numel(lambda_values),1);
    for i = 1:numel(lambda_values)
        labels{i} = sprintf('\\lambda=%.1f', ...
            lambda_values(i));
    end

    x = 1:numel(lambda_values);

    plot_bar_error( ...
        nexttile, x, summary.RMSE_mean, ...
        summary.RMSE_std, labels, 'RMSE', ...
        'Across-seed tracking RMSE');

    plot_bar_error( ...
        nexttile, x, summary.ISE_mean, ...
        summary.ISE_std, labels, ...
        'ISE (a.u.^2 h)', ...
        'Across-seed tracking ISE');

    ax3 = nexttile;
    hold(ax3,'on');

    Y = [ ...
        summary.ISE_mean, ...
        summary.ISE_CVaR, ...
        summary.ISE_worst];

    bar(ax3, x, Y, 'grouped');
    set(ax3,'XTick',x,'XTickLabel',labels);
    ylabel(ax3,'ISE (a.u.^2 h)');
    title(ax3,sprintf( ...
        'Mean, empirical CVaR_{%.2f}, and worst ISE', ...
        opts.robust_cvar_q));
    legend(ax3, ...
        {'Mean','Upper-tail CVaR','Worst seed'}, ...
        'Location','best');
    grid(ax3,'on');

    plot_bar_error( ...
        nexttile, x, summary.TimeInBand_mean, ...
        summary.TimeInBand_std, labels, ...
        'Time in \pm5% band (%)', ...
        'Fraction of simulation within target band');

    set(findall(fig,'Type','axes'), ...
        'FontSize',15, ...
        'LineWidth',1.1, ...
        'Box','on');

    drawnow;

    base = fullfile(opts.output_dir, ...
        'rapid_cvar_ablation_metrics');
    save_figure_files(fig, base, opts);

    close(fig);
end


function plot_bar_error( ...
        ax, x, values, errors, labels, yLabel, ttl)

    hold(ax,'on');
    bar(ax, x, values, 0.65);
    errorbar(ax, x, values, errors, 'k.', ...
        'LineWidth',1.5, ...
        'CapSize',10);

    set(ax,'XTick',x,'XTickLabel',labels);
    ylabel(ax,yLabel);
    title(ax,ttl);
    grid(ax,'on');
end


function label = lambda_label(lambda_cvar)
    if abs(lambda_cvar-0.6) < 1e-12
        label = '\lambda=0.6 (mean--CVaR)';
    elseif abs(lambda_cvar) < 1e-12
        label = '\lambda=0 (mean only)';
    else
        label = sprintf('\\lambda=%.3g',lambda_cvar);
    end
end


function save_figure_files(fig, baseName, opts)
    if opts.save_pdf
        try
            exportgraphics(fig, [baseName '.pdf'], ...
                'ContentType','vector');
        catch ME
            %warning('Vector PDF export failed: %s', ME.message);
            warning(ME.identifier, ...
                'Vector PDF export failed: %s', ME.message);
            print(fig, [baseName '.pdf'], ...
                '-dpdf', '-bestfit');
        end
    end

    if opts.save_png
        try
            exportgraphics(fig, [baseName '.png'], ...
                'Resolution',300);
        catch ME
            %warning('PNG export failed: %s', ME.message);
            warning(ME.identifier, ...
                'PNG export failed: %s', ME.message);
            print(fig, [baseName '.png'], ...
                '-dpng', '-r300');
        end
    end
end


function write_latex_table(summary, filename, q)
    fid = fopen(filename, 'w');

    if fid < 0
        warning('Could not open %s for writing.', filename);
        return;
    end

    %cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
    cleanup = onCleanup(@() fclose(fid));

    fprintf(fid, ...
        '%% Automatically generated by run_rapid_cvar_ablation_safe.m\n');
    fprintf(fid, '\\begin{table}[!t]\n');
    fprintf(fid, '\\centering\n');
    fprintf(fid, ...
        ['\\caption{Ablation of the CVaR component in RAPID. ' ...
        'Values are reported across paired random seeds as mean ' ...
        '$\\pm$ standard deviation. The empirical upper-tail CVaR ' ...
        'of the realized ISE uses $q=%.2f$.}\n'], q);
    fprintf(fid, '\\label{tab:rapid_cvar_ablation}\n');
    fprintf(fid, '\\small\n');
    fprintf(fid, '\\begin{tabular}{lccccc}\n');
    fprintf(fid, '\\hline\n');
    fprintf(fid, ...
        ['$\\lambda$ & RMSE & ISE & CVaR$_{%.2f}$(ISE) & ' ...
        'Overshoot & Time in band (\\%%) \\\\\n'], q);
    fprintf(fid, '\\hline\n');

    for i = 1:height(summary)
        fprintf(fid, ...
            ['%.1f & %.3g $\\pm$ %.2g & ' ...
            '%.3g $\\pm$ %.2g & %.3g & ' ...
            '%.3g $\\pm$ %.2g & %.2f $\\pm$ %.2f \\\\\n'], ...
            summary.Lambda(i), ...
            summary.RMSE_mean(i), ...
            summary.RMSE_std(i), ...
            summary.ISE_mean(i), ...
            summary.ISE_std(i), ...
            summary.ISE_CVaR(i), ...
            summary.Overshoot_mean(i), ...
            summary.Overshoot_std(i), ...
            summary.TimeInBand_mean(i), ...
            summary.TimeInBand_std(i));
    end

    fprintf(fid, '\\hline\n');
    fprintf(fid, '\\end{tabular}\n');
    fprintf(fid, '\\end{table}\n');
end
