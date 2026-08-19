function rapid_plot_summary(OUT, CTRL, opts)
%RAPID_PLOT_SUMMARY Readability-revised RAPID summary figure.
%
% Plotting-only changes:
%   1. Larger fonts/line widths for manuscript readability.
%   2. A--F panel labels placed outside the axes boxes.
%   3. Extra headroom in tracking, threshold, and amplitude panels.
%   4. PID gains normalized by their respective initial values.
%   5. Normalized gain panel uses a logarithmic y-axis when all plotted
%      gain ratios are positive, with distinct line styles/markers so
%      Kp, Ki, and Kd remain distinguishable despite different scales.
%
% No controller or simulation calculations are changed.

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
    T4_meas_valid = T4_meas_used(mask_meas);

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

    fig = figure( ...
        'Color','w', ...
        'Position',[100 100 1800 1250]);

    FS_axes   = 22;
    FS_label  = 24;
    FS_title  = 25;
    FS_legend = 18;
    FS_panel  = 26;

    LW        = 2.5;
    LW_ref    = 2.2;
    MS        = 5;

    % A. Tracking
    ax1 = subplot(3,2,1);
    hold(ax1,'on');

    patch(ax1, ...
        [t_hr_p, fliplr(t_hr_p)], ...
        [band_lo*ones(size(t_hr_p)), fliplr(band_hi*ones(size(t_hr_p)))], ...
        [0.92 0.92 0.92], ...
        'EdgeColor','none', ...
        'FaceAlpha',0.5, ...
        'DisplayName','\pm5% band');

    plot(ax1, t_hr_p, T4_des*ones(size(t_hr_p)), '--k', ...
        'LineWidth',LW_ref, 'DisplayName','Setpoint');

    plot(ax1, t_hr_p, T4_log_p, '-r', ...
        'LineWidth',LW, 'DisplayName','T_4^{ext} true');

    plot(ax1, t_meas_hr, T4_meas_valid, '-o', ...
        'Color',[0.85 0.50 0], ...
        'LineWidth',LW-0.4, ...
        'MarkerSize',MS, ...
        'DisplayName','Used measurement');

    xlabel(ax1,'Time (h)','FontSize',FS_label);
    yl1 = ylabel(ax1,'T_4^{ext} (a.u.)','FontSize',FS_label);
    title(ax1,'Tracking','FontSize',FS_title);
    grid(ax1,'on');

    trackVals = [T4_log_p(:); T4_meas_valid(:); T4_des; band_hi; band_lo];
    trackVals = trackVals(isfinite(trackVals));
    if ~isempty(trackVals)
        trackMin = min(trackVals);
        trackMax = max(trackVals);
        trackSpan = max(trackMax-trackMin,1);
        ylim(ax1,[max(0,trackMin-0.08*trackSpan), trackMax+0.30*trackSpan]);
    end

    lgd1 = legend(ax1,'Location','southeast','FontSize',FS_legend);
    lgd1.Box = 'on';

    % B. PID command
    ax2 = subplot(3,2,2);
    hold(ax2,'on');

    plot(ax2,t_hr_p,u_pid_p,'-b', ...
        'LineWidth',LW,'DisplayName','u_{PID}');

    yline(ax2,CTRL.thresh,'--', ...
        'Color',[0.85 0.33 0.10], ...
        'LineWidth',LW_ref, ...
        'DisplayName','threshold');

    pidValid = u_pid_p(isfinite(u_pid_p));
    pidVals = [pidValid(:); CTRL.thresh];
    if ~isempty(pidVals)
        pidLo = min(pidVals);
        pidHi = max(pidVals);
        pidSpan = max(pidHi-pidLo,max(1e-6,0.05*max(abs(pidVals))));
        ylim(ax2,[pidLo-0.08*pidSpan, pidHi+0.08*pidSpan]);
    end

    xlabel(ax2,'Time (h)','FontSize',FS_label);
    yl2 = ylabel(ax2,'u_{PID} (a.u.)','FontSize',FS_label);
    title(ax2,'PID command','FontSize',FS_title);
    grid(ax2,'on');

    lgd2 = legend(ax2,'Location','best','FontSize',FS_legend);
    lgd2.Box = 'on';

    % C. EF supplied
    ax3 = subplot(3,2,3);
    hold(ax3,'on');

    plot(ax3,t_hr_p,EF_p,'m','LineWidth',LW);

    xlabel(ax3,'Time (h)','FontSize',FS_label);
    yl3 = ylabel(ax3,'EF supplied (a.u.)','FontSize',FS_label);
    title(ax3,'EF with true delay/jitter','FontSize',FS_title);
    grid(ax3,'on');

    % D. Per-window amplitude
    ax4 = subplot(3,2,4);
    hold(ax4,'on');

    stairs(ax4,sample_times(maskA)/60,A_hist(maskA), ...
        'LineWidth',LW,'DisplayName','A_{cmd}');

    yline(ax4,CTRL.A_max,'--r', ...
        'LineWidth',LW_ref,'DisplayName','A_{max}');

    Avalid = A_hist(maskA);
    Afinite = Avalid(isfinite(Avalid));
    ampVals = [Afinite(:); CTRL.A_max; 0];
    if ~isempty(ampVals)
        ampLo = min(ampVals);
        ampHi = max(ampVals);
        ampSpan = max(ampHi-ampLo,max(1e-6,0.05*max(abs(ampVals))));
        ylim(ax4,[max(0,ampLo-0.08*ampSpan), ampHi+0.10*ampSpan]);
    end

    xlabel(ax4,'Time (h)','FontSize',FS_label);
    yl4 = ylabel(ax4,'Amplitude A (a.u.)','FontSize',FS_label);
    title(ax4,'Per-window amplitude','FontSize',FS_title);
    grid(ax4,'on');

    lgd4 = legend(ax4,'Location','best','FontSize',FS_legend);
    lgd4.Box = 'on';

    % E. Normalized adaptive PID gains
    ax5 = subplot(3,2,5);
    hold(ax5,'on');

    i0Kp = find(~isnan(Kp_hist),1,'first');
    i0Ki = find(~isnan(Ki_hist),1,'first');
    i0Kd = find(~isnan(Kd_hist),1,'first');

    if isempty(i0Kp) || isempty(i0Ki) || isempty(i0Kd)
        error('PID gain histories contain no valid initial values.');
    end

    Kp0 = Kp_hist(i0Kp);
    Ki0 = Ki_hist(i0Ki);
    Kd0 = Kd_hist(i0Kd);

    Kp_rel = Kp_hist / max(abs(Kp0),eps);
    Ki_rel = Ki_hist / max(abs(Ki0),eps);
    Kd_rel = Kd_hist / max(abs(Kd0),eps);

    stairs(ax5,sample_times/60,Kp_rel,'-o', ...
        'LineWidth',LW,'MarkerSize',MS,'DisplayName','K_p/K_{p,0}');

    stairs(ax5,sample_times/60,Ki_rel,'--s', ...
        'LineWidth',LW,'MarkerSize',MS,'DisplayName','K_i/K_{i,0}');

    stairs(ax5,sample_times/60,Kd_rel,'-.^', ...
        'LineWidth',LW,'MarkerSize',MS,'DisplayName','K_d/K_{d,0}');

    yline(ax5,1,'--k','LineWidth',1.2,'HandleVisibility','off');

    KpFinite = Kp_rel(isfinite(Kp_rel));
    KiFinite = Ki_rel(isfinite(Ki_rel));
    KdFinite = Kd_rel(isfinite(Kd_rel));
    gainVals = [KpFinite(:); KiFinite(:); KdFinite(:)];

    if ~isempty(gainVals) && all(gainVals > 0)
        set(ax5,'YScale','log');

        logMin = log10(min(gainVals));
        logMax = log10(max(gainVals));
        logSpan = max(logMax-logMin,0.20);

        yLo = 10^(logMin-0.10*logSpan);
        yHi = 10^(logMax+0.10*logSpan);
        ylim(ax5,[yLo yHi]);
    else
        set(ax5,'YScale','linear');

        gainValsLinear = [gainVals(:); 1];
        gainLo = min(gainValsLinear);
        gainHi = max(gainValsLinear);
        gainSpan = max(gainHi-gainLo,max(1e-6,0.05*max(abs(gainValsLinear))));

        ylim(ax5,[max(0,gainLo-0.08*gainSpan), gainHi+0.10*gainSpan]);
    end

    xlabel(ax5,'Time (h)','FontSize',FS_label);
    yl5 = ylabel(ax5,'Gain / initial gain','FontSize',FS_label);
    title(ax5,'Normalized adaptive PID gains','FontSize',FS_title);
    grid(ax5,'on');

    lgd5 = legend(ax5,'Location','best','FontSize',FS_legend);
    lgd5.Box = 'on';

    % F. Per-window robust cost
    ax6 = subplot(3,2,6);
    hold(ax6,'on');

    stairs(ax6,sample_times(maskJ)/60,cost_hist(maskJ),'LineWidth',LW);

    xlabel(ax6,'Time (h)','FontSize',FS_label);
    yl6 = ylabel(ax6,'J_{rob}','FontSize',FS_label);
    title(ax6,'Per-window robust cost','FontSize',FS_title);
    grid(ax6,'on');

    % Common formatting
    allAxes = [ax1 ax2 ax3 ax4 ax5 ax6];

    set(allAxes, ...
        'FontSize',FS_axes, ...
        'LineWidth',1.4, ...
        'Box','on', ...
        'TickDir','out');

    yl1.FontSize = FS_label;
    yl2.FontSize = FS_label;
    yl3.FontSize = FS_label;
    yl4.FontSize = FS_label;
    yl5.FontSize = FS_label;
    yl6.FontSize = FS_label;

    linkaxes(allAxes,'x');

    % External A--F labels
    drawnow;
    add_panel_label_outside(fig,ax1,'A',FS_panel);
    add_panel_label_outside(fig,ax2,'B',FS_panel);
    add_panel_label_outside(fig,ax3,'C',FS_panel);
    add_panel_label_outside(fig,ax4,'D',FS_panel);
    add_panel_label_outside(fig,ax5,'E',FS_panel);
    add_panel_label_outside(fig,ax6,'F',FS_panel);

    if opts.verbose
        fprintf('\nInitial RAPID gains used for normalization:\n');
        fprintf('Kp0 = %.8g\n',Kp0);
        fprintf('Ki0 = %.8g\n',Ki0);
        fprintf('Kd0 = %.8g\n',Kd0);

        fprintf('\nMaximum normalized gains:\n');
        fprintf('max(Kp/Kp0) = %.6g\n',max(KpFinite));
        fprintf('max(Ki/Ki0) = %.6g\n',max(KiFinite));
        fprintf('max(Kd/Kd0) = %.6g\n',max(KdFinite));
    end

    if opts.do_save_pdf
        if ~exist(opts.output_dir,'dir')
            mkdir(opts.output_dir);
        end

        pdfFile = fullfile( ...
            opts.output_dir, ...
            sprintf('rapid_clean5_setpoint_%g.pdf',T4_des));

        drawnow;
        exportgraphics(fig,pdfFile, ...
            'ContentType','vector', ...
            'BackgroundColor','white');

        if opts.verbose
            fprintf('\nSaved RAPID figure:\n%s\n',pdfFile);
        end
    end
end


function h = add_panel_label_outside(fig,ax,labelText,FS_panel)
%ADD_PANEL_LABEL_OUTSIDE Place label outside upper-left corner of axes.

    drawnow;

    oldUnits = ax.Units;
    ax.Units = 'normalized';
    pos = ax.Position;
    ax.Units = oldUnits;

    x = pos(1) - 0.040;
    y = pos(2) + pos(4) + 0.006;

    x = max(0.005,x);
    y = min(0.965,y);

    h = annotation(fig,'textbox', ...
        [x y 0.035 0.035], ...
        'String',labelText, ...
        'FontSize',FS_panel, ...
        'FontWeight','bold', ...
        'HorizontalAlignment','left', ...
        'VerticalAlignment','bottom', ...
        'EdgeColor','none', ...
        'BackgroundColor','none', ...
        'Margin',0, ...
        'FitBoxToText','on');
end
