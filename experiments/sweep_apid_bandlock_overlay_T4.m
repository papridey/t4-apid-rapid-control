function RUNS = sweep_apid_bandlock_overlay_T4(setpoints, opts)
%SWEEP_APID_BANDLOCK_OVERLAY_T4
% Run the APID + band-lock controller for multiple setpoints and generate
% overlay plots for:
%   (1) continuous simulated T4 trajectories
%   (2) measured T4 trajectories
%
% Main simulator:
%   run_apid_bandlock_smooth.m
%
% Usage:
%   RUNS = sweep_apid_bandlock_overlay_T4();
%   RUNS = sweep_apid_bandlock_overlay_T4([15 20 25 30 35 45]);
%   RUNS = sweep_apid_bandlock_overlay_T4([15 20 25 30 35 45], struct('save_plot', true));
%
% Optional inputs:
%   setpoints : vector of desired T4 setpoints
%   opts      : struct with optional fields
%       .save_plot   (default: true)
%       .output_dir  (default: pwd)
%       .make_plots  (default: false)   % passed to run_apid_bandlock_smooth
%       .verbose     (default: true)
%       .dt_seconds  (default: 0.1)
%       .sim_hours   (default: 80)

    if nargin < 1 || isempty(setpoints)
        setpoints = [15 20 25 30 35 45];
    end
    if nargin < 2
        opts = struct();
    end

    opts = set_default(opts, 'save_plot', true);
    opts = set_default(opts, 'output_dir', pwd);
    opts = set_default(opts, 'make_plots', false);
    opts = set_default(opts, 'verbose', true);
    opts = set_default(opts, 'dt_seconds', 0.1);
    opts = set_default(opts, 'sim_hours', 80);

    if ~exist(opts.output_dir, 'dir')
        mkdir(opts.output_dir);
    end

    validateattributes(setpoints, {'numeric'}, ...
        {'vector', 'real', 'finite', 'positive'}, mfilename, 'setpoints', 1);

    setpoints = setpoints(:).';
    Nset = numel(setpoints);
    RUNS = cell(1, Nset);

    if opts.verbose
        fprintf('\n=== APID setpoint sweep ===\n');
        fprintf('Setpoints: %s\n', mat2str(setpoints));
        fprintf('Output directory: %s\n\n', opts.output_dir);
    end

    % -------------------------------------------------------------
    % Run one APID simulation for each setpoint
    % -------------------------------------------------------------
    for i = 1:Nset
        T4_des = setpoints(i);

        if opts.verbose
            fprintf('================ RUN %d / %d : setpoint = %g ================\n', ...
                i, Nset, T4_des);
        end

        sim_opts = struct();
        sim_opts.make_plots  = opts.make_plots;
        sim_opts.verbose     = opts.verbose;
        sim_opts.dt_seconds  = opts.dt_seconds;
        sim_opts.sim_hours   = opts.sim_hours;
        sim_opts.output_dir  = opts.output_dir;
        sim_opts.do_save_pdf = false;

        RUNS{i} = run_apid_bandlock_smooth(T4_des, sim_opts);
    end

    % -------------------------------------------------------------
    % Common plot settings
    % -------------------------------------------------------------
    FS_axes   = 20;
    FS_label  = 20;
    FS_title  = 24;
    FS_legend = 16;
    LW_main   = 2.0;
    MS_main   = 5;

    % -------------------------------------------------------------
    % Continuous T4 overlay plot
    % -------------------------------------------------------------
    fig_cont = figure('Color','w','Position',[120 120 1000 620]);
    hold on;

    for i = 1:Nset
        if ~isfield(RUNS{i}, 'time_hr') || ~isfield(RUNS{i}, 'T4_log')
            warning('Run %d is missing time_hr or T4_log. Skipping continuous plot.', i);
            continue;
        end

        t_cont_hr = RUNS{i}.time_hr(:);
        T4_cont_i = RUNS{i}.T4_log(:);
        sp        = RUNS{i}.T4_des;

        mask_cont = ~isnan(t_cont_hr) & ~isnan(T4_cont_i);

        plot(t_cont_hr(mask_cont), T4_cont_i(mask_cont), '-', ...
            'LineWidth', LW_main, ...
            'DisplayName', sprintf('T_4^{ext} (sp = %g)', sp));

        if any(mask_cont)
            t0 = t_cont_hr(find(mask_cont,1,'first'));
            tf = t_cont_hr(find(mask_cont,1,'last'));
            plot([t0, tf], [sp, sp], '--k', ...
                'LineWidth', 0.8, ...
                'HandleVisibility', 'off');
        end
    end

    xlabel('Time (h)', 'FontSize', FS_label);
    ylabel('T_4^{ext} (a.u.)', 'FontSize', FS_label);
    title('T_4 Tracking Overlay for Multiple Setpoints', 'FontSize', FS_title);
    grid on;

    ax = gca;
    ax.FontSize = FS_axes;

    lgd = legend('Location', 'eastoutside');
    lgd.FontSize = FS_legend;

    % -------------------------------------------------------------
    % Measured T4 overlay plot
    % -------------------------------------------------------------
    fig_meas = figure('Color','w','Position',[150 150 1000 620]);
    hold on;

    for i = 1:Nset
        if ~isfield(RUNS{i}, 't_meas') || ~isfield(RUNS{i}, 'T4_meas')
            warning('Run %d is missing t_meas or T4_meas. Skipping measured plot.', i);
            continue;
        end

        mask_meas = ~isnan(RUNS{i}.t_meas) & ~isnan(RUNS{i}.T4_meas);
        t_meas_hr = RUNS{i}.t_meas(mask_meas) / 60;
        T4_meas_i = RUNS{i}.T4_meas(mask_meas);
        sp        = RUNS{i}.T4_des;

        plot(t_meas_hr, T4_meas_i, '-o', ...
            'LineWidth', LW_main, ...
            'MarkerSize', MS_main, ...
            'DisplayName', sprintf('T_4^{meas} (sp = %g)', sp));

        if ~isempty(t_meas_hr)
            plot([t_meas_hr(1), t_meas_hr(end)], [sp, sp], '--k', ...
                'LineWidth', 0.8, ...
                'HandleVisibility', 'off');
        end
    end

    xlabel('Time (h)', 'FontSize', FS_label);
    ylabel('Measured T_4^{ext} (a.u.)', 'FontSize', FS_label);
    title('Measured T_4 Tracking Overlay for Multiple Setpoints', 'FontSize', FS_title);
    grid on;

    ax = gca;
    ax.FontSize = FS_axes;

    lgd = legend('Location', 'eastoutside');
    lgd.FontSize = FS_legend;

    % -------------------------------------------------------------
    % Save plots
    % -------------------------------------------------------------
    if opts.save_plot
        figname_cont = 'sweep_apid_bandlock_T4_continuous_overlay';
        figname_meas = 'sweep_apid_bandlock_T4_measured_overlay';

        exportgraphics(fig_cont, fullfile(opts.output_dir, [figname_cont '.pdf']), ...
            'ContentType', 'vector');
        exportgraphics(fig_cont, fullfile(opts.output_dir, [figname_cont '.png']), ...
            'Resolution', 300);

        exportgraphics(fig_meas, fullfile(opts.output_dir, [figname_meas '.pdf']), ...
            'ContentType', 'vector');
        exportgraphics(fig_meas, fullfile(opts.output_dir, [figname_meas '.png']), ...
            'Resolution', 300);

        if opts.verbose
            fprintf('\nSaved overlay figures:\n');
            fprintf('  %s\n', fullfile(opts.output_dir, [figname_cont '.pdf']));
            fprintf('  %s\n', fullfile(opts.output_dir, [figname_cont '.png']));
            fprintf('  %s\n', fullfile(opts.output_dir, [figname_meas '.pdf']));
            fprintf('  %s\n', fullfile(opts.output_dir, [figname_meas '.png']));
        end
    end
end

% =====================================================================
% local helper
% =====================================================================

function s = set_default(s, name, value)
    if ~isfield(s, name) || isempty(s.(name))
        s.(name) = value;
    end
end