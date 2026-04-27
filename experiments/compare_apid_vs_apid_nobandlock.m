function compare_apid_vs_apid_nobandlock(T4_des)

    if nargin < 1 || isempty(T4_des)
        T4_des = 25;
    end

    opts = struct();
    opts.dt_seconds  = 0.1;
    opts.sim_hours   = 80;
    opts.make_plots  = false;
    opts.verbose     = false;
    opts.do_save_pdf = false;

    OUT_lock   = run_apid_bandlock_smooth(T4_des, opts);
    OUT_nolock = run_apid_nobandlock_smooth(T4_des, opts);

    M_lock   = compute_metrics(OUT_lock);
    M_nolock = compute_metrics(OUT_nolock);

    fprintf('\n================ Metrics ================\n');
    fprintf('Band-lock APID    : IAE = %.4f | ITAE = %.4f | Time-in-5%% = %.2f%% | Final meas = %.4f\n', ...
        M_lock.IAE, M_lock.ITAE, 100*M_lock.time_in_5, M_lock.final_meas);
    fprintf('No-band-lock APID : IAE = %.4f | ITAE = %.4f | Time-in-5%% = %.2f%% | Final meas = %.4f\n', ...
        M_nolock.IAE, M_nolock.ITAE, 100*M_nolock.time_in_5, M_nolock.final_meas);

    % measured samples only
    m1 = ~isnan(OUT_lock.t_meas)   & ~isnan(OUT_lock.T4_meas);
    m2 = ~isnan(OUT_nolock.t_meas) & ~isnan(OUT_nolock.T4_meas);

    t1 = OUT_lock.t_meas(m1)/60;
    y1 = OUT_lock.T4_meas(m1);

    t2 = OUT_nolock.t_meas(m2)/60;
    y2 = OUT_nolock.T4_meas(m2);

    % 5% band
    tmax = max([t1(:); t2(:); 0]);
    tband = linspace(0, tmax, 400);
    band_hi = 1.05*T4_des;
    band_lo = 0.95*T4_des;

    figure('Color','w','Position',[200 150 900 500]); hold on;

    patch([tband fliplr(tband)], ...
          [band_lo*ones(size(tband)) fliplr(band_hi*ones(size(tband)))], ...
          [0.92 0.92 0.92], 'EdgeColor','none', 'FaceAlpha',0.5, ...
          'DisplayName','\pm5% band');

    plot(tband, T4_des*ones(size(tband)), '--k', 'LineWidth',1.5, ...
         'DisplayName','Setpoint');

    plot(t1, y1, '-o', 'LineWidth',1.8, 'MarkerSize',5, ...
         'DisplayName','APID with band-lock');

    plot(t2, y2, '-s', 'LineWidth',1.8, 'MarkerSize',5, ...
         'DisplayName','APID without band-lock');

    xlabel('Time (h)');
    ylabel('Measured T_4^{ext} (a.u.)');
    title(sprintf('Measured T_4 comparison only (T4_{des}=%.2f)', T4_des));
    grid on;
    legend('Location','best');
end

% ============================================================
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