function rapid_plot_summary(OUT, CTRL, opts)

    time         = OUT.time;
    time_hr      = OUT.time_hr;
    T4_log       = OUT.T4_log;
    T4_des       = OUT.T4_des;
    u_pid        = OUT.u_pid;
    EF_log       = OUT.EF_supplied_log;
    sample_times = OUT.sample_times;
    A_hist       = OUT.A_hist;
    Kp_hist      = OUT.Kp_hist;
    Ki_hist      = OUT.Ki_hist;
    Kd_hist      = OUT.Kd_hist;
    cost_hist    = OUT.cost_hist;
    t_meas       = OUT.t_meas;
    T4_meas_used = OUT.T4_meas_used;

    mask_meas = ~isnan(t_meas) & ~isnan(T4_meas_used);
    t_meas_hr = t_meas(mask_meas) / 60;

    maskA = ~isnan(A_hist);
    maskJ = ~isnan(cost_hist);

    Nplot_max = 6000;
    stride = max(1, ceil(numel(time) / Nplot_max));
    idxp = 1:stride:numel(time);

    t_hr_p   = time_hr(idxp);
    T4_log_p = T4_log(idxp);
    u_pid_p  = u_pid(idxp);
    EF_p     = EF_log(idxp);

    band_pct_plot = 0.05;
    band_hi = T4_des * (1 + band_pct_plot);
    band_lo = T4_des * (1 - band_pct_plot);

    fig = figure('Color','w','Position',[100 100 1500 1100]);

    FS_axes   = 20;
    FS_label  = 20;
    FS_title  = 24;
    FS_legend = 18;
    LW        = 2.0;
    MS        = 5;

    %----------------------%
    % 1. Tracking
    %----------------------%
    ax1 = subplot(3,2,1); hold(ax1,'on');
    patch(ax1, [t_hr_p, fliplr(t_hr_p)], ...
          [band_lo*ones(size(t_hr_p)), fliplr(band_hi*ones(size(t_hr_p)))], ...
          [0.92 0.92 0.92], ...
          'EdgeColor','none', 'FaceAlpha',0.5, ...
          'DisplayName','\pm5% band');

    plot(ax1, t_hr_p, T4_des*ones(size(t_hr_p)), '--k', ...
         'LineWidth', LW, 'DisplayName', 'Setpoint');

    plot(ax1, t_hr_p, T4_log_p, '-r', ...
         'LineWidth', LW, 'DisplayName', 'T_4^{ext} true');

    plot(ax1, t_meas_hr, T4_meas_used(mask_meas), '-', ...
         'Color', [0.85 0.5 0], 'LineWidth', LW-0.4, ...
         'Marker', 'o', 'MarkerSize', MS, ...
         'DisplayName', 'Used measurement');

    xlabel(ax1, 'Time (h)', 'FontSize', FS_label);
    ylabel(ax1, 'T_4^{ext} (a.u.)', 'FontSize', FS_label);
    title(ax1, 'Tracking', 'FontSize', FS_title);
    legend(ax1, 'Location', 'best', 'FontSize', FS_legend);
    grid(ax1, 'on');
    set(ax1, 'FontSize', FS_axes, 'LineWidth', 1.2, 'Box', 'on');

    %----------------------%
    % 2. PID command
    %----------------------%
    ax2 = subplot(3,2,2); hold(ax2,'on');
    plot(ax2, t_hr_p, u_pid_p, '-b', ...
         'LineWidth', LW, 'DisplayName', 'u_{PID}');

    yline(ax2, CTRL.thresh, '--', ...
          'Color', [1 0.5 0], 'LineWidth', 1.6, ...
          'DisplayName', 'threshold');

    xlabel(ax2, 'Time (h)', 'FontSize', FS_label);
    ylabel(ax2, 'u_{PID} (a.u.)', 'FontSize', FS_label);
    title(ax2, 'PID command', 'FontSize', FS_title);
    legend(ax2, 'Location', 'best', 'FontSize', FS_legend);
    grid(ax2, 'on');
    set(ax2, 'FontSize', FS_axes, 'LineWidth', 1.2, 'Box', 'on');

    %----------------------%
    % 3. EF supplied
    %----------------------%
    ax3 = subplot(3,2,3); hold(ax3,'on');
    plot(ax3, t_hr_p, EF_p, 'm', 'LineWidth', LW);

    xlabel(ax3, 'Time (h)', 'FontSize', FS_label);
    ylabel(ax3, 'EF supplied (a.u.)', 'FontSize', FS_label);
    title(ax3, 'EF with true delay/jitter', 'FontSize', FS_title);
    grid(ax3, 'on');
    set(ax3, 'FontSize', FS_axes, 'LineWidth', 1.2, 'Box', 'on');

    %----------------------%
    % 4. Per-window amplitude
    %----------------------%
    ax4 = subplot(3,2,4); hold(ax4,'on');
    stairs(ax4, sample_times(maskA)/60, A_hist(maskA), ...
           'LineWidth', LW, 'DisplayName', 'A_{cmd}');

    yline(ax4, CTRL.A_max, '--r', ...
          'LineWidth', 1.6, 'DisplayName', 'A_{max}');

    xlabel(ax4, 'Time (h)', 'FontSize', FS_label);
    ylabel(ax4, 'Amplitude A (a.u.)', 'FontSize', FS_label);
    title(ax4, 'Per-window amplitude', 'FontSize', FS_title);
    legend(ax4, 'Location', 'best', 'FontSize', FS_legend);
    grid(ax4, 'on');
    set(ax4, 'FontSize', FS_axes, 'LineWidth', 1.2, 'Box', 'on');

    %----------------------%
    % 5. Adaptive PID gains
    %----------------------%
    ax5 = subplot(3,2,5); hold(ax5,'on');

    i0Kp = find(~isnan(Kp_hist), 1, 'first');
    i0Ki = find(~isnan(Ki_hist), 1, 'first');
    i0Kd = find(~isnan(Kd_hist), 1, 'first');

    stairs(ax5, sample_times/60, Kp_hist, ...
           'LineWidth', LW, ...
           'DisplayName', sprintf('K_p (init = %.3g)', Kp_hist(i0Kp)));

    stairs(ax5, sample_times/60, Ki_hist, ...
           'LineWidth', LW, ...
           'DisplayName', sprintf('K_i (init = %.3g)', Ki_hist(i0Ki)));

    stairs(ax5, sample_times/60, Kd_hist, ...
           'LineWidth', LW, ...
           'DisplayName', sprintf('K_d (init = %.3g)', Kd_hist(i0Kd)));

    xlabel(ax5, 'Time (h)', 'FontSize', FS_label);
    ylabel(ax5, 'Gains', 'FontSize', FS_label);
    title(ax5, 'Adaptive PID gains', 'FontSize', FS_title);
    legend(ax5, 'Location', 'best', 'FontSize', FS_legend);
    grid(ax5, 'on');
    set(ax5, 'FontSize', FS_axes, 'LineWidth', 1.2, 'Box', 'on');

    %----------------------%
    % 6. Robust cost
    %----------------------%
    ax6 = subplot(3,2,6); hold(ax6,'on');
    stairs(ax6, sample_times(maskJ)/60, cost_hist(maskJ), ...
           'LineWidth', LW);

    xlabel(ax6, 'Time (h)', 'FontSize', FS_label);
    ylabel(ax6, 'J_{rob}', 'FontSize', FS_label);
    title(ax6, 'Per-window robust cost', 'FontSize', FS_title);
    grid(ax6, 'on');
    set(ax6, 'FontSize', FS_axes, 'LineWidth', 1.2, 'Box', 'on');

    %----------------------%
    % Link x-axes
    %----------------------%
    linkaxes([ax1 ax2 ax3 ax4 ax5 ax6], 'x');

    %----------------------%
    % Optional PDF save
    %----------------------%
    if opts.do_save_pdf
        print(fig, fullfile(opts.output_dir, ...
            sprintf('rapid_clean5_setpoint_%g.pdf', T4_des)), ...
            '-dpdf', '-bestfit');
    end
end