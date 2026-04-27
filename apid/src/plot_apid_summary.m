function plot_apid_summary(OUT, thresh, A_max, opts)
%PLOT_APID_SUMMARY Summary plots for APID runs.

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
    T4_meas      = OUT.T4_meas;

    mask_meas = ~isnan(t_meas) & ~isnan(T4_meas);
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
    FS_legend = 16;
    LW        = 2.0;
    MS        = 5;

    ax1 = subplot(3,2,1); hold(ax1,'on');
    %patch(ax1, [t_hr_p, fliplr(t_hr_p)], ...
        %  [band_lo*ones(size(t_hr_p)), fliplr(band_hi*ones(size(t_hr_p)))], ...
         % [0.92 0.92 0.92], 'EdgeColor','none', 'FaceAlpha',0.5, ...
         % 'DisplayName','\pm 5\% band');
    patch(ax1, [t_hr_p, fliplr(t_hr_p)], ...
      [band_lo*ones(size(t_hr_p)), fliplr(band_hi*ones(size(t_hr_p)))], ...
      [0.92 0.92 0.92], 'EdgeColor','none', 'FaceAlpha',0.5, ...
      'DisplayName','±5% band');
    plot(ax1, t_hr_p, T4_des*ones(size(t_hr_p)), '--k', ...
        'LineWidth',LW, 'DisplayName','Setpoint');
    plot(ax1, t_hr_p, T4_log_p, '-r', 'LineWidth',LW, ...
        'DisplayName','T_4^{ext} (continuous)');
    plot(ax1, t_meas_hr, T4_meas(mask_meas), '-', 'Color',[0.85 0.5 0], ...
        'LineWidth',1.0, 'DisplayName','Measured');
    plot(ax1, t_meas_hr, T4_meas(mask_meas), 'o', 'Color',[0.25 0.25 0.25], ...
        'MarkerFaceColor',[0.25 0.25 0.25], 'MarkerSize',MS, ...
        'DisplayName','Measurement instants');

    xlabel(ax1,'Time (h)','FontSize',FS_label);
    %yl1 = ylabel(ax1,'$T_4^{\mathrm{ext}}$ (a.u.)', ...
        %'FontSize',FS_label,'Interpreter','latex');
    yl1 = ylabel(ax1,'T_4^{ext} (a.u.)', ...
        'FontSize',FS_label);
    title(ax1,'Tracking','FontSize',FS_title);
    set(ax1,'FontSize',FS_axes);
    yl1.FontSize = FS_label;
    grid(ax1,'on');

    %lgd1 = legend(ax1,'Location','southeast','FontSize',FS_legend,'Interpreter','latex');
    lgd1 = legend(ax1,'Location','southeast','FontSize',FS_legend);
    lgd1.Box = 'on';

    ax2 = subplot(3,2,2); hold(ax2,'on');
    plot(ax2, t_hr_p, u_pid_p, '-b', 'LineWidth',1.3, 'DisplayName','u_{PID}');
    yline(ax2, thresh, '--', 'Color',[1 0.5 0], 'LineWidth',1.1, 'DisplayName','threshold');
    xlabel(ax2,'Time (h)','FontSize',FS_label);
    yl2 = ylabel(ax2,'u_{PID} (a.u.)','FontSize',FS_label);
    title(ax2,'PID command','FontSize',FS_title);
    set(ax2,'FontSize',FS_axes);
    yl2.FontSize = FS_label;
    grid(ax2,'on');
    legend(ax2,'Location','best','FontSize',FS_legend);

    ax3 = subplot(3,2,3); hold(ax3,'on');
    plot(ax3, t_hr_p, EF_p, 'm', 'LineWidth',1.3);
    xlabel(ax3,'Time (h)','FontSize',FS_label);
    yl3 = ylabel(ax3,'EF (avg, bursts; a.u.)','FontSize',FS_label);
    title(ax3,'EF supplied','FontSize',FS_title);
    set(ax3,'FontSize',FS_axes);
    yl3.FontSize = FS_label;
    grid(ax3,'on');

    ax4 = subplot(3,2,4); hold(ax4,'on');
    stairs(ax4, sample_times(maskA)/60, A_hist(maskA), 'LineWidth',1.4, 'DisplayName','A_{applied}');
    yline(ax4, A_max, '--r', 'LineWidth',1.1, 'DisplayName','A_{max}');
    xlabel(ax4,'Time (h)','FontSize',FS_label);
    yl4 = ylabel(ax4,'A_{applied} (a.u.)','FontSize',FS_label);
    title(ax4,'Per-window amplitude','FontSize',FS_title);
    set(ax4,'FontSize',FS_axes);
    yl4.FontSize = FS_label;
    grid(ax4,'on');
    legend(ax4,'Location','best','FontSize',FS_legend);

    ax5 = subplot(3,2,5); hold(ax5,'on');
    i0Kp = find(~isnan(Kp_hist),1,'first');
    i0Ki = find(~isnan(Ki_hist),1,'first');
    i0Kd = find(~isnan(Kd_hist),1,'first');
    Kp0 = Kp_hist(i0Kp);
    Ki0 = Ki_hist(i0Ki);
    Kd0 = Kd_hist(i0Kd);

    stairs(ax5, sample_times/60, Kp_hist, 'LineWidth',1.4, ...
        'DisplayName', sprintf('K_p (init = %.3g)', Kp0));
    stairs(ax5, sample_times/60, Ki_hist, 'LineWidth',1.4, ...
        'DisplayName', sprintf('K_i (init = %.3g)', Ki0));
    stairs(ax5, sample_times/60, Kd_hist, 'LineWidth',1.4, ...
        'DisplayName', sprintf('K_d (init = %.3g)', Kd0));
    xlabel(ax5,'Time (h)','FontSize',FS_label);
    yl5 = ylabel(ax5,'Gains','FontSize',FS_label);
    title(ax5,'Adaptive PID gains','FontSize',FS_title);
    set(ax5,'FontSize',FS_axes);
    yl5.FontSize = FS_label;
    ax5.YAxis.Exponent = -3;
    grid(ax5,'on');
    legend(ax5,'Location','best','FontSize',FS_legend);

    ax6 = subplot(3,2,6); hold(ax6,'on');
    stairs(ax6, sample_times(maskJ)/60, cost_hist(maskJ), 'LineWidth',1.2);
    xlabel(ax6,'Time (h)','FontSize',FS_label);
    yl6 = ylabel(ax6,'$J_m$','FontSize',FS_label,'Interpreter','latex');
    title(ax6,'Per-window cost','FontSize',FS_title);
    set(ax6,'FontSize',FS_axes);
    yl6.FontSize = FS_label;
    ax6.YAxis.Exponent = 4;
    grid(ax6,'on');

    set([ax1,ax2,ax3,ax4,ax5,ax6],'FontSize',FS_axes);
    linkaxes([ax1,ax2,ax3,ax4,ax5,ax6],'x');

    if opts.verbose
        fprintf('\n=== Tracking diagnostics ===\n');
        fprintf('N full points           = %d\n', numel(time));
        fprintf('N plotted points        = %d\n', numel(idxp));
        fprintf('Max continuous T4       = %.6f\n', max(T4_log));
        fprintf('Min continuous error    = %.6f\n', min(T4_des - T4_log));

        if any(mask_meas)
            fprintf('Max sampled T4          = %.6f\n', max(T4_meas(mask_meas)));
            fprintf('Min sampled error       = %.6f\n', min(T4_des - T4_meas(mask_meas)));
        end
    end
     

    Nzoom = numel(idxp);
    i1 = max(1, round(0.15 * Nzoom));
    i2 = min(Nzoom, round(0.30 * Nzoom));

    figure('Color','w','Position',[820 120 650 300]);
    plot(t_hr_p(i1:i2), EF_p(i1:i2), 'm', 'LineWidth',1.3);
    xlabel('Time (h)');
    ylabel('EF (avg, bursts)', 'FontSize',FS_label);
    title('EF supplied (zoomed subset)', 'FontSize',FS_title);
    grid on;

    %Nzoom = numel(idxp);
    %i1 = max(1, round(0.15 * Nzoom));
    %i2 = min(Nzoom, round(0.30 * Nzoom));

    %fig_zoom = figure('Color','w','Position',[820 120 650 300]);
    %axz = axes(fig_zoom); hold(axz,'on');
    %plot(axz, t_hr_p(i1:i2), EF_p(i1:i2), 'm', 'LineWidth',1.3);

    %xlabel(axz,'Time (h)','FontSize',FS_label);
    %ylz = ylabel(axz,'EF (avg, bursts; a.u.)','FontSize',FS_label);
    %title(axz,'EF supplied (zoomed subset)','FontSize',FS_title);

    %set(axz,'FontSize',FS_axes);
    %ylz.FontSize = FS_label;
    %grid(axz,'on');

    %ymin = min(EF_p(i1:i2));
    %ymax = max(EF_p(i1:i2));
    %pad  = 0.08*(ymax - ymin + eps);
    %ylim(axz,[ymin-pad, ymax+pad]);
    %yticks(axz, linspace(ymin, ymax, 5));
    %ytickformat(axz,'%.4f');

    if opts.do_save_pdf
        set(fig,'PaperPositionMode','auto');
        drawnow;
        print(fig, fullfile(opts.output_dir, sprintf('apid_setpoint_%g.pdf', OUT.T4_des)), '-dpdf', '-bestfit');
    end
end