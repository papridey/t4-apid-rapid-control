function OUT = run_apid_bandlock_smooth(T4_des, opts)
%RUN_APID_BANDLOCK_SMOOTH
% Windowed adaptive PID with band-lock holding for EF-responsive T4 control.
%
% This refactored version separates the main simulation loop from APID-specific
% helper functions, while relying on shared utilities in ../common.
%
% Example:
%   OUT = run_apid_bandlock_smooth(25);
%   OUT = run_apid_bandlock_smooth(25, struct('dt_seconds', 0.1/12));

    if nargin < 1 || isempty(T4_des)
        T4_des = 25;
    end
    if nargin < 2
        opts = struct();
    end

    this_dir = fileparts(mfilename('fullpath'));
    repo_dir = fileparts(this_dir);
    addpath(fullfile(this_dir, 'src'));
    addpath(fullfile(repo_dir, 'common', 'utils'));
    addpath(fullfile(repo_dir, 'common', 'timing'));
    addpath(fullfile(repo_dir, 'common', 'plant'));

    opts = get_apid_defaults(opts);

    if ~exist(opts.output_dir, 'dir')
        mkdir(opts.output_dir);
    end

    timing = get_apid_timing_and_schedule(opts);
    ctrl   = get_apid_controller_params();
    P      = get_constant_plant_params();

    y_log = zeros(16, timing.Nsteps);
    y0 = [zeros(15,1); 25];
    y_log(:,1) = y0;

    T4_log = zeros(1, timing.Nsteps);
    T4_log(1) = y0(4);

    u_pid           = zeros(1, timing.Nsteps);
    EF_supplied_log = zeros(1, timing.Nsteps);
    A_t             = zeros(1, timing.Nsteps);

    A_hist    = nan(1, numel(timing.sample_times));
    t_meas    = nan(size(timing.sample_times));
    T4_meas   = nan(size(timing.sample_times));
    Kp_hist   = nan(size(timing.sample_times));
    Ki_hist   = nan(size(timing.sample_times));
    Kd_hist   = nan(size(timing.sample_times));
    cost_hist = nan(size(timing.sample_times));
    lock_hist = false(size(timing.sample_times));

    state = initialize_apid_state(T4_des, T4_log(1), ctrl);

    A_hist(1)    = state.A_hold;
    Kp_hist(1)   = state.Kp;
    Ki_hist(1)   = state.Ki;
    Kd_hist(1)   = state.Kd;
    cost_hist(1) = 0;
    t_meas(1)    = timing.time(1);
    T4_meas(1)   = T4_log(1);
    lock_hist(1) = false;

    m = 2;

    for k = 1:timing.Nsteps-1
        tk = timing.time(k);

        [A_now, EF_time_k_min] = apid_window_actuation(state.A_hold, timing, ctrl);
        A_t(k) = A_now;

        u_pid(k)           = state.EF_cmd;
        EF_supplied_log(k) = EF_time_k_min(tk);

        odefun = @(t_min, y) ode_constant_coeff_model(t_min, y, EF_time_k_min, P);
        [~, Y] = ode15s(odefun, [timing.time(k), timing.time(k+1)], y_log(:,k));
        y_log(:,k+1) = Y(end,:)';
        T4_log(k+1)  = y_log(4,k+1);

        if m <= numel(timing.sample_times) && timing.time(k+1) >= timing.sample_times(m)
            yk = y_log(4,k+1);
            T4_meas(m) = yk;
            t_meas(m)  = timing.time(k+1);

            prev_y_for_cost       = state.prev_y;
            dmeas_f_prev_for_cost = state.dmeas_f;

            dmeas = (yk - state.prev_y) / timing.Ts;
            state.dmeas_f = (1 - ctrl.alpha_d) * state.dmeas_f + ctrl.alpha_d * dmeas;
            state.prev_y = yk;

            e_meas = T4_des - yk;
            e_true = T4_des - yk;

            state = update_apid_bandlock_state(state, e_true, ctrl, T4_des);

            cost_baseline = window_cost_ISE_apid( ...
                y_log(:,k+1), state.Kp, state.Ki, state.Kd, T4_des, yk, ...
                prev_y_for_cost, dmeas_f_prev_for_cost, state.int_e, timing.Ts, ...
                u_pid(max(k,1)), state.A_hold, state.A_basal, state.first_band_hit, ...
                ctrl.EF_min, ctrl.EF_max, ctrl.EF_rate, ctrl.Kb_aw, ...
                ctrl.thresh, ctrl.kA, ctrl.A_min, ctrl.A_max, ctrl.dA_max, ctrl.A_filter_fac, ...
                timing.T_total, timing.T_cycle, timing.T_on, timing.time(k+1), ...
                timing.pulseDuty, timing.t5, timing.t6, ctrl.rhoA, ctrl.beta_ov, P, ctrl.alpha_d);

            if state.band_locked
                [EF_cmd_next, A_next, state] = apid_lock_mode_step(state, e_true, ctrl);
            else
                [EF_cmd_next, A_next, state] = apid_adaptation_step( ...
                    y_log(:,k+1), yk, T4_des, prev_y_for_cost, dmeas_f_prev_for_cost, ...
                    timing, ctrl, P, state, u_pid(max(k,1)), timing.time(k+1));
            end

            state.EF_cmd = EF_cmd_next;
            state.A_hold = A_next;

            Kp_hist(m)   = state.Kp;
            Ki_hist(m)   = state.Ki;
            Kd_hist(m)   = state.Kd;
            cost_hist(m) = cost_baseline;
            A_hist(m)    = state.A_hold;
            lock_hist(m) = state.band_locked;

            if opts.verbose
                fprintf(['win %2d t=%6.0f min | y=%.2f e=%.2f | ', ...
                         'lock=%d | Kp=%.3g Ki=%.3g Kd=%.3g | J=%.3g | A=%.6f | Abasal=%.6f\n'], ...
                         m-1, timing.time(k+1), yk, e_meas, state.band_locked, ...
                         state.Kp, state.Ki, state.Kd, cost_baseline, state.A_hold, state.A_basal);
            end

            state.prev_e = e_meas;
            m = m + 1;
        end
    end

    u_pid(end)           = state.EF_cmd;
    EF_supplied_log(end) = EF_supplied_log(max(end-1,1));
    A_t(end)             = A_t(max(end-1,1));

    OUT.time            = timing.time;
    OUT.time_hr         = timing.time / 60;
    OUT.T4_log          = T4_log;
    OUT.T4_des          = T4_des;
    OUT.u_pid           = u_pid;
    OUT.EF_supplied_log = EF_supplied_log;
    OUT.A_t             = A_t;
    OUT.sample_times    = timing.sample_times;
    OUT.A_hist          = A_hist;
    OUT.Kp_hist         = Kp_hist;
    OUT.Ki_hist         = Ki_hist;
    OUT.Kd_hist         = Kd_hist;
    OUT.cost_hist       = cost_hist;
    OUT.t_meas          = t_meas;
    OUT.T4_meas         = T4_meas;
    OUT.lock_hist       = lock_hist;
    OUT.EF_min          = ctrl.EF_min;
    OUT.EF_max          = ctrl.EF_max;
    OUT.opts            = opts;
    OUT.params          = P;

    if opts.make_plots
        plot_apid_summary(OUT, ctrl.thresh, ctrl.A_max, opts);
    end
end
