function OUT = run_rapid_clean5(T4_des, opts)
%RUN_RAPID_CLEAN5 Robust adaptive PID controller for EF-responsive T4 regulation.
%   OUT = RUN_RAPID_CLEAN5(T4_des)
%   OUT = RUN_RAPID_CLEAN5(T4_des, opts)
%
%

    if nargin < 1 || isempty(T4_des)
        T4_des = 25;
    end
    if nargin < 2
        opts = struct();
    end

    repoRoot = fileparts(mfilename('fullpath'));
    srcPath = fullfile(repoRoot, 'src');
    if exist(srcPath, 'dir')
        addpath(srcPath);
    end

    opts = rapid_apply_default_options(opts);
    if ~exist(opts.output_dir, 'dir')
        mkdir(opts.output_dir);
    end

    CONST = rapid_constants();
    SCHED = rapid_build_schedule(opts, CONST);
    CTRL  = rapid_controller_defaults(T4_des, SCHED);
    MEAS  = rapid_measurement_defaults();
    P     = rapid_get_nominal_plant_params();
    [PERT_TRUE, ROB] = rapid_build_robust_configs(opts, MEAS, CONST);
    streams = rapid_make_streams(opts.rng_seed);
    pert_true = rapid_sample_perturbation(PERT_TRUE.range, PERT_TRUE.d4_period_min, streams.truePert);

    y_log = zeros(16, SCHED.Nsteps);
    y0 = [zeros(15,1); 25];
    y_log(:,1) = y0;

    T4_log = zeros(1, SCHED.Nsteps);
    T4_log(1) = y0(4);

    u_pid           = zeros(1, SCHED.Nsteps);
    EF_supplied_log = zeros(1, SCHED.Nsteps);
    A_t             = zeros(1, SCHED.Nsteps);

    A_hist         = nan(1, numel(SCHED.sample_times));
    t_meas         = nan(size(SCHED.sample_times));
    T4_meas_raw    = nan(size(SCHED.sample_times));
    T4_meas_used   = nan(size(SCHED.sample_times));
    bias_hat_hist  = nan(size(SCHED.sample_times));
    Kp_hist        = nan(size(SCHED.sample_times));
    Ki_hist        = nan(size(SCHED.sample_times));
    Kd_hist        = nan(size(SCHED.sample_times));
    cost_hist      = nan(size(SCHED.sample_times));
    lock_hist      = false(size(SCHED.sample_times));

    state = rapid_init_controller_state(T4_des, T4_log(1), CTRL, MEAS, CONST);

    A_hist(1)        = state.A_hold;
    Kp_hist(1)       = state.Kp;
    Ki_hist(1)       = state.Ki;
    Kd_hist(1)       = state.Kd;
    cost_hist(1)     = 0;
    t_meas(1)        = SCHED.time(1);
    T4_meas_raw(1)   = T4_log(1);
    T4_meas_used(1)  = T4_log(1);
    bias_hat_hist(1) = state.meas_state.bhat;
    lock_hist(1)     = false;

    m = 2;

    for k = 1:SCHED.Nsteps-1
        tk = SCHED.time(k);
        state.r_f = state.r_f + (SCHED.dt / CTRL.tau_ref) * (T4_des - state.r_f);

        [A_now, EF_time_k_min] = rapid_build_window_input(state.A_hold, pert_true, SCHED, CTRL);
        A_t(k) = A_now;
        u_pid(k) = state.EF_cmd;
        EF_supplied_log(k) = EF_time_k_min(tk);

        odefun = @(t_min, y) rapid_ode_perturbed_model(t_min, y, EF_time_k_min, pert_true, P);
        [~, Y] = ode15s(odefun, [SCHED.time(k), SCHED.time(k+1)], y_log(:,k));
        y_log(:,k+1) = Y(end,:)';
        T4_log(k+1)  = y_log(4,k+1);

        if m <= numel(SCHED.sample_times) && SCHED.time(k+1) >= SCHED.sample_times(m)
            y_true = y_log(4,k+1);

            if MEAS.enable
                y_raw = y_true + MEAS.bias_true + MEAS.sigma * randn(streams.meas);
            else
                y_raw = y_true;
            end

            [y_used, state.meas_state] = rapid_measurement_filter_and_bias_correct(y_raw, state.r_f, state.meas_state, MEAS);

            T4_meas_raw(m)   = y_raw;
            T4_meas_used(m)  = y_used;
            bias_hat_hist(m) = state.meas_state.bhat;
            t_meas(m)        = SCHED.time(k+1);

            dmeas = (y_used - state.prev_y_used) / SCHED.Ts;
            state.dmeas_f = (1 - CTRL.alpha_d) * state.dmeas_f + CTRL.alpha_d * dmeas;
            state.prev_y_used = y_used;

            e_meas = state.r_f - y_used;
            e_true = T4_des - y_used;

            [state.band_locked, state.first_band_hit, state.A_basal, state.int_e] = ...
                rapid_update_bandlock(state.band_locked, state.first_band_hit, state.A_basal, ...
                                      state.A_hold, state.int_e, e_true, T4_des, CTRL);

            scen = rapid_sample_perturbations(ROB.range, ROB.d4_period_min, ROB.M, streams.scen);
            for i = 1:numel(scen)
                scen(i).meas_sigma = ROB.meas_sigma;
                scen(i).meas_bias  = ROB.meas_bias;
                scen(i).noise_seed = streams.costBase + 1000*m + i;
            end

            cost_baseline = rapid_window_cost_ISE_robust( ...
                y_log(:,k+1), state.Kp, state.Ki, state.Kd, state.r_f, y_used, state.int_e, state.prev_e, SCHED.Ts, ...
                state.EF_cmd, state.A_hold, CTRL, SCHED, SCHED.time(k+1), scen, ROB, P);

            if state.band_locked
                [state.EF_cmd, state.A_hold, state.A_basal] = rapid_bandlock_step( ...
                    e_true, state.A_hold, state.A_basal, CTRL);
            else
                    [state.Kp, state.Ki, state.Kd, state.int_e, state.EF_cmd, state.A_hold] = ...
                    rapid_adaptive_pid_step(y_log(:,k+1), state.Kp, state.Ki, state.Kd, ...
                    state.r_f, y_used, state.int_e, state.prev_e, state.dmeas_f, ...
                    SCHED, SCHED.time(k+1), state.EF_cmd, state.A_hold, ...
                    state.first_band_hit, state.A_basal, CTRL, scen, ROB, P, e_meas);
            end

            Kp_hist(m)   = state.Kp;
            Ki_hist(m)   = state.Ki;
            Kd_hist(m)   = state.Kd;
            cost_hist(m) = cost_baseline;
            A_hist(m)    = state.A_hold;
            lock_hist(m) = state.band_locked;

            if opts.verbose
                fprintf(['win %2d t=%6.0f min | y_true=%.2f y_used=%.2f e=%.2f | ', ...
                         'lock=%d | Kp=%.3g Ki=%.3g Kd=%.3g | Jrob=%.3g | A=%.6f | Abasal=%.6f | bhat=%.2f\n'], ...
                         m-1, SCHED.time(k+1), y_true, y_used, e_meas, state.band_locked, ...
                         state.Kp, state.Ki, state.Kd, cost_baseline, state.A_hold, state.A_basal, state.meas_state.bhat);
            end

            state.prev_e = e_meas;
            m = m + 1;
        end
    end

    u_pid(end)           = state.EF_cmd;
    EF_supplied_log(end) = EF_supplied_log(max(end-1,1));
    A_t(end)             = A_t(max(end-1,1));

    OUT.time            = SCHED.time;
    OUT.time_hr         = SCHED.time / 60;
    OUT.T4_log          = T4_log;
    OUT.T4_des          = T4_des;
    OUT.u_pid           = u_pid;
    OUT.EF_supplied_log = EF_supplied_log;
    OUT.A_t             = A_t;
    OUT.sample_times    = SCHED.sample_times;
    OUT.A_hist          = A_hist;
    OUT.Kp_hist         = Kp_hist;
    OUT.Ki_hist         = Ki_hist;
    OUT.Kd_hist         = Kd_hist;
    OUT.cost_hist       = cost_hist;
    OUT.t_meas          = t_meas;
    OUT.T4_meas_raw     = T4_meas_raw;
    OUT.T4_meas_used    = T4_meas_used;
    OUT.bias_hat_hist   = bias_hat_hist;
    OUT.lock_hist       = lock_hist;
    OUT.pert_true       = pert_true;
    OUT.masterSeed      = opts.rng_seed;
    OUT.opts            = opts;
    OUT.params          = P;
    OUT.schedule        = SCHED;
    OUT.controller      = CTRL;
    OUT.measurement     = MEAS;

    if opts.do_plots
        rapid_plot_summary(OUT, CTRL, opts);
    end
end
