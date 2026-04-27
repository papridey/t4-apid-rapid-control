function RUNS = open_loop_T4_amplitude_sweep_overlay(A_list, opts)
%OPEN_LOOP_T4_AMPLITUDE_SWEEP_OVERLAY
% Sweep fixed open-loop amplitudes A and generate one overlay plot of T4.
%
% This script uses the same burst-averaged EF configuration and the same
% constant-coefficient 16-state plant model as the closed-loop APID code,
% but without feedback.
%
% Usage:
%   RUNS = open_loop_T4_amplitude_sweep_overlay();
%   RUNS = open_loop_T4_amplitude_sweep_overlay([0 0.005 0.01 0.015 0.02]);
%   RUNS = open_loop_T4_amplitude_sweep_overlay([0 0.005 0.01 0.015 0.02], ...
%              struct('save_plot', true, 'dt_seconds', 0.1));
%
% Optional inputs:
%   A_list : vector of fixed amplitudes
%   opts   : struct with optional fields
%       .save_plot   (default: false)
%       .output_dir  (default: pwd)
%       .verbose     (default: true)
%       .dt_seconds  (default: 0.1)
%       .sim_hours   (default: 80)

    if nargin < 1 || isempty(A_list)
        A_list = [0 0.005 0.01 0.015 0.02];
    end
    if nargin < 2
        opts = struct();
    end

    opts = set_default(opts, 'save_plot', false);
    opts = set_default(opts, 'output_dir', pwd);
    opts = set_default(opts, 'verbose', true);
    opts = set_default(opts, 'dt_seconds', 0.1);
    opts = set_default(opts, 'sim_hours', 80);

    if ~exist(opts.output_dir, 'dir')
        mkdir(opts.output_dir);
    end

    validateattributes(A_list, {'numeric'}, ...
        {'vector', 'real', 'finite', 'nonnegative'}, mfilename, 'A_list', 1);

    A_list = A_list(:).';
    Nrun = numel(A_list);
    RUNS = cell(1, Nrun);

    if opts.verbose
        fprintf('\n=== Open-loop amplitude sweep ===\n');
        fprintf('Amplitudes: %s\n', mat2str(A_list));
        fprintf('Output directory: %s\n\n', opts.output_dir);
    end

    for i = 1:Nrun
        A_fixed = A_list(i);

        if opts.verbose
            fprintf('================ OPEN-LOOP RUN %d / %d : A = %.4g ================\n', ...
                i, Nrun, A_fixed);
        end

        RUNS{i} = run_one_open_loop_fixed_A(A_fixed, opts);
    end

    % -------------------------------------------------------------
    % Overlay plot
    % -------------------------------------------------------------
    fig1 = figure('Color','w','Position',[120 120 1000 620]);
    hold on;

    FS_axes   = 20;
    FS_label  = 20;
    FS_title  = 24;
    FS_legend = 20;
    LW_main   = 1.5;

    for i = 1:Nrun
        plot(RUNS{i}.time_hr, RUNS{i}.T4_log, 'LineWidth', LW_main, ...
            'DisplayName', sprintf('A = %.3g', RUNS{i}.A_fixed));
    end

    xlabel('Time (h)', 'FontSize', FS_label);
    ylabel('T_4^{ext} (a.u.)', 'FontSize', FS_label);
    title('Open-loop T_4 Trajectories for Fixed Amplitudes', 'FontSize', FS_title);
    grid on;

    ax = gca;
    ax.FontSize = FS_axes;

    lgd = legend('Location','eastoutside');
    lgd.FontSize = FS_legend;

    if opts.save_plot
        figname = 'open_loop_T4_amplitude_sweep_overlay';
        exportgraphics(fig1, fullfile(opts.output_dir, [figname '.pdf']), ...
            'ContentType', 'vector');
        exportgraphics(fig1, fullfile(opts.output_dir, [figname '.png']), ...
            'Resolution', 300);

        if opts.verbose
            fprintf('\nSaved open-loop overlay figure:\n');
            fprintf('  %s\n', fullfile(opts.output_dir, [figname '.pdf']));
            fprintf('  %s\n', fullfile(opts.output_dir, [figname '.png']));
        end
    end
end

% =====================================================================
% one open-loop run
% =====================================================================

function OUT = run_one_open_loop_fixed_A(A_fixed, opts)

    % -------------------------------------------------------------
    % Unit conversion
    % -------------------------------------------------------------
    SEC_PER_MIN = 60;
    toMin = @(sec) sec / SEC_PER_MIN;

    % -------------------------------------------------------------
    % Simulation horizon (minutes)
    % -------------------------------------------------------------
    dt   = toMin(opts.dt_seconds);
    ti   = 0;
    tf   = opts.sim_hours * 60;
    time = ti:dt:tf;
    Nsteps = numel(time);

    % -------------------------------------------------------------
    % Window schedule (minutes)
    % -------------------------------------------------------------
    Ts      = 120;
    r       = 0.60;
    T_on    = r * Ts;
    T_cycle = Ts;
    T_total = tf;

    Nw           = ceil((tf - ti) / Ts);
    sample_times = ti + (0:Nw) * Ts;
    sample_times(end) = tf;

    % -------------------------------------------------------------
    % Bursts & micro-pulse timing (minutes)
    % -------------------------------------------------------------
    t1 = toMin(500e-6);
    t2 = toMin(100e-6);
    t3 = toMin(500e-6);
    t4 = toMin(9.5e-4);

    Tp        = t1 + t2 + t3 + t4;
    pulseDuty = (t1 + t3) / Tp;

    np = 500;
    t5 = np * Tp;
    t6 = toMin(0.5);

    % -------------------------------------------------------------
    % Amplitude limits
    % -------------------------------------------------------------
    A_min = 0.00;
    A_max = 0.10;
    A_hold = clamp(A_fixed, A_min, A_max);

    % -------------------------------------------------------------
    % Constant plant parameters
    % -------------------------------------------------------------
    P = get_constant_plant_params();

    % -------------------------------------------------------------
    % State and logs
    % -------------------------------------------------------------
    y_log = zeros(16, Nsteps);
    y0 = [zeros(15,1); 25];
    y_log(:,1) = y0;

    T4_log = zeros(1, Nsteps);
    T4_log(1) = y0(4);

    EF_supplied_log = zeros(1, Nsteps);
    A_t = zeros(1, Nsteps);

    t_meas = nan(size(sample_times));
    T4_meas = nan(size(sample_times));
    m = 1;

    % -------------------------------------------------------------
    % Open-loop simulation
    % -------------------------------------------------------------
    for k = 1:Nsteps-1
        tk = time(k);

        if A_hold <= 0
            A_now = 0;
            EF_time_k_min = @(t_min) 0.0;
        else
            A_now = A_hold;
            EF_time_k_min = @(t_min) ef_avg_burst_in_window( ...
                t_min, T_on, T_cycle, T_total, t5, t6, A_hold, pulseDuty);
        end

        A_t(k) = A_now;
        EF_supplied_log(k) = EF_time_k_min(tk);

        odefun = @(t_min, y) ode_constant_coeff_model(t_min, y, EF_time_k_min, P);
        [~, Y] = ode15s(odefun, [time(k), time(k+1)], y_log(:,k));
        y_log(:,k+1) = Y(end,:)';
        T4_log(k+1)  = y_log(4,k+1);

        if m <= numel(sample_times) && time(k+1) >= sample_times(m)
            t_meas(m)  = time(k+1);
            T4_meas(m) = y_log(4,k+1);
            m = m + 1;
        end
    end

    EF_supplied_log(end) = EF_supplied_log(max(end-1,1));
    A_t(end)             = A_t(max(end-1,1));

    % -------------------------------------------------------------
    % Pack outputs
    % -------------------------------------------------------------
    OUT.time            = time;
    OUT.time_hr         = time / 60;
    OUT.T4_log          = T4_log;
    OUT.A_fixed         = A_fixed;
    OUT.t_meas          = t_meas;
    OUT.T4_meas         = T4_meas;
    OUT.EF_supplied_log = EF_supplied_log;
    OUT.A_t             = A_t;
    OUT.max_T4          = max(T4_log);
    OUT.final_T4        = T4_log(end);
    OUT.opts            = opts;
    OUT.params          = P;

    if opts.verbose
        fprintf('A = %.4g | max T4_ext = %.4g | final T4_ext = %.4g\n', ...
            OUT.A_fixed, OUT.max_T4, OUT.final_T4);
    end
end

% =====================================================================
% local helpers
% =====================================================================

function P = get_constant_plant_params()
    P.alpha1      = 3.308;
    P.gammaext_T4 = 0.01;
    P.M_fun       = 0.15;

    P.alpha  = 20 * 8.43812;
    P.K      = 10 * 0.1195;
    P.gamma1 = 0.0296;
    P.n      = 2;

    P.N = 10;
    P.a = 0.0001 * P.N;

    P.alpha_NKX21_offset = 0.25;
    P.gamma_NKX21        = 0.01;

    P.kappa = 1;
    P.K2    = 1000;

    P.alpha_TG    = 1;
    P.alpha_I     = 1;
    P.gamma_I     = 0.004;
    P.gamma_Tg    = 0.04;
    P.gammaint_T4 = 0.15 + 0.0004;
end

function s = set_default(s, name, value)
    if ~isfield(s, name) || isempty(s.(name))
        s.(name) = value;
    end
end