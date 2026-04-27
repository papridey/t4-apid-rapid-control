function compare_apid_vs_apid_nobandlock_all(T4_des)

    if nargin < 1 || isempty(T4_des)
        T4_des = 25;
    end

    opts = struct();
    opts.dt_seconds  = 0.1;
    opts.sim_hours   = 80;
    opts.make_plots  = false;
    opts.verbose     = false;   % avoid command-window slowdown
    opts.do_save_pdf = false;

    OUT_lock   = run_apid_bandlock_smooth(T4_des, opts);
    OUT_nolock = run_apid_nobandlock_smooth(T4_des, opts);

    M_lock   = compute_metrics(OUT_lock);
    M_nolock = compute_metrics(OUT_nolock);

    fprintf('\n================ Metrics ================\n');
    fprintf('Band-lock APID      : IAE = %.4f | ITAE = %.4f | Time-in-5%% = %.2f%% | Final meas = %.4f\n', ...
        M_lock.IAE, M_lock.ITAE, 100*M_lock.time_in_5, M_lock.final_meas);
    fprintf('No-band-lock APID   : IAE = %.4f | ITAE = %.4f | Time-in-5%% = %.2f%% | Final meas = %.4f\n', ...
        M_nolock.IAE, M_nolock.ITAE, 100*M_nolock.time_in_5, M_nolock.final_meas);

    % ---------------------------------------------------------
    % downsample continuous trajectories for plotting only
    % ---------------------------------------------------------
    Nplot_max = 6000;

    idx1 = 1:max(1,ceil(numel(OUT_lock.time_hr)/Nplot_max)):numel(OUT_lock.time_hr);
    idx2 = 1:max(1,ceil(numel(OUT_nolock.time_hr)/Nplot_max)):numel(OUT_nolock.time_hr);

    t1c  = OUT_lock.time_hr(idx1);
    y1c  = OUT_lock.T4_log(idx1);
    u1c  = OUT_lock.u_pid(idx1);

    t2c  = OUT_nolock.time_hr(idx2);
    y2c  = OUT_nolock.T4_log(idx2);
    u2c  = OUT_nolock.u_pid(idx2);

    % measured data
    m1 = ~isnan(OUT_lock.t_meas)   & ~isnan(OUT_lock.T4_meas);
    m2 = ~isnan(OUT_nolock.t_meas) & ~isnan(OUT_nolock.T4_meas);

    t1m = OUT_lock.t_meas(m1)/60;
    y1m = OUT_lock.T4_meas(m1);

    t2m = OUT_nolock.t_meas(m2)/60;
    y2m = OUT_nolock.T4_meas(m2);

    figure('Color','w','Position',[100 80 1500 1000]);

    % ---------------------------------------------------------
    % 1. measured T4
    % ---------------------------------------------------------
    subplot(3,2,1); hold on;
    band_hi = 1.05*T4_des;
    band_lo = 0.95*T4_des;

    tpatch = linspace(0, max([OUT_lock.time_hr(end), OUT_nolock.time_hr(end)]), 1000);
    patch([tpatch, fliplr(tpatch)], ...
          [band_lo*ones(size(tpatch)), fliplr(band_hi*ones(size(tpatch)))], ...
          [0.92 0.92 0.92], 'EdgeColor','none', 'FaceAlpha',0.5);

    plot(tpatch, T4_des*ones(size(tpatch)), '--k', 'LineWidth',1.5, 'DisplayName','Setpoint');
    plot(t1m, y1m, '-o', 'LineWidth',1.6, 'MarkerSize',4, 'DisplayName','APID with band-lock');
    plot(t2m, y2m, '-s', 'LineWidth',1.6, 'MarkerSize',4, 'DisplayName','APID without band-lock');
    xlabel('Time (h)');
    ylabel('Measured T_4^{ext}');
    title('Measured tracking comparison');
    grid on;
    legend('Location','best');

    % ---------------------------------------------------------
    % 2. continuous T4
    % ---------------------------------------------------------
    subplot(3,2,2); hold on;
    plot(t1c, y1c, 'LineWidth',1.8, 'DisplayName','APID with band-lock');
    plot(t2c, y2c, '--', 'LineWidth',1.8, 'DisplayName','APID without band-lock');
    plot(tpatch, T4_des*ones(size(tpatch)), ':k', 'LineWidth',1.5, 'DisplayName','Setpoint');
    xlabel('Time (h)');
    ylabel('Continuous T_4^{ext}');
    title('Continuous trajectory');
    grid on;
    legend('Location','best');

    % ---------------------------------------------------------
    % 3. amplitude
    % ---------------------------------------------------------
    subplot(3,2,3); hold on;
    stairs(OUT_lock.sample_times/60, OUT_lock.A_hist, 'LineWidth',1.5, 'DisplayName','with band-lock');
    stairs(OUT_nolock.sample_times/60, OUT_nolock.A_hist, '--', 'LineWidth',1.5, 'DisplayName','without band-lock');
    xlabel('Time (h)');
    ylabel('A');
    title('Per-window amplitude');
    grid on;
    legend('Location','best');

    % ---------------------------------------------------------
    % 4. PID command
    % ---------------------------------------------------------
    subplot(3,2,4); hold on;
    plot(t1c, u1c, 'LineWidth',1.4, 'DisplayName','with band-lock');
    plot(t2c, u2c, '--', 'LineWidth',1.4, 'DisplayName','without band-lock');
    xlabel('Time (h)');
    ylabel('u_{PID}');
    title('PID command');
    grid on;
    legend('Location','best');

    % ---------------------------------------------------------
    % 5. Gains
    % ---------------------------------------------------------
    subplot(3,2,5); hold on;
    stairs(OUT_lock.sample_times/60, OUT_lock.Kp_hist, 'LineWidth',1.3, 'DisplayName','K_p with lock');
    stairs(OUT_nolock.sample_times/60, OUT_nolock.Kp_hist, '--', 'LineWidth',1.3, 'DisplayName','K_p no lock');
    stairs(OUT_lock.sample_times/60, OUT_lock.Ki_hist, 'LineWidth',1.3, 'DisplayName','K_i with lock');
    stairs(OUT_nolock.sample_times/60, OUT_nolock.Ki_hist, '--', 'LineWidth',1.3, 'DisplayName','K_i no lock');
    stairs(OUT_lock.sample_times/60, OUT_lock.Kd_hist, 'LineWidth',1.3, 'DisplayName','K_d with lock');
    stairs(OUT_nolock.sample_times/60, OUT_nolock.Kd_hist, '--', 'LineWidth',1.3, 'DisplayName','K_d no lock');
    xlabel('Time (h)');
    ylabel('Gain values');
    title('Adaptive gains');
    grid on;
    legend('Location','eastoutside');

    % ---------------------------------------------------------
    % 6. Cost
    % ---------------------------------------------------------
    subplot(3,2,6); hold on;
    stairs(OUT_lock.sample_times/60, OUT_lock.cost_hist, 'LineWidth',1.4, 'DisplayName','with band-lock');
    stairs(OUT_nolock.sample_times/60, OUT_nolock.cost_hist, '--', 'LineWidth',1.4, 'DisplayName','without band-lock');
    xlabel('Time (h)');
    ylabel('J_k');
    title('Per-window predictive cost');
    grid on;
    legend('Location','best');

    sgtitle(sprintf('APID with band-lock vs APID without band-lock (T4_{des} = %.2f)', T4_des), ...
        'FontWeight','bold');
end

% =====================================================================
function M = compute_metrics(OUT)

    mask = ~isnan(OUT.t_meas) & ~isnan(OUT.T4_meas);
    t = OUT.t_meas(mask);
    y = OUT.T4_meas(mask);
    r = OUT.T4_des * ones(size(y));

    e = r - y;

    if numel(t) >= 2
        M.IAE  = trapz(t, abs(e));
        M.ITAE = trapz(t, t .* abs(e));
    else
        M.IAE  = NaN;
        M.ITAE = NaN;
    end

    band5 = 0.05 * max(OUT.T4_des, 1e-8);
    M.time_in_5 = mean(abs(e) <= band5);

    M.final_meas = y(end);
    M.max_meas   = max(y);
    M.min_meas   = min(y);
end