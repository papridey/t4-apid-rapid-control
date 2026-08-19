function plot_apid_summary(OUT, thresh, A_max, opts)
%PLOT_APID_SUMMARY Summary plots for APID runs.
%
% Readability-revised version for manuscript figures.
%
% Changes are plotting-only:
%   1. Larger fonts and line widths.
%   2. External A--F panel labels.
%   3. Additional vertical headroom in tracking panel.
%   4. More visible threshold and A_max lines.
%   5. Adaptive gains normalized by their initial values:
%          Kp/Kp0, Ki/Ki0, Kd/Kd0.
%   6. Cleaner measurement representation in panel A.
%
% No controller or simulation calculations are changed.

    % ================================================================
    % Unpack output
    % ================================================================
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

    % ================================================================
    % Masks
    % ================================================================
    mask_meas = ~isnan(t_meas) & ~isnan(T4_meas);
    t_meas_hr = t_meas(mask_meas) / 60;
    T4_meas_valid = T4_meas(mask_meas);

    maskA = ~isnan(A_hist);
    maskJ = ~isnan(cost_hist);

    % ================================================================
    % Downsample only for plotting continuous trajectories
    % ================================================================
    Nplot_max = 6000;

    stride = max(1, ceil(numel(time) / Nplot_max));
    idxp = 1:stride:numel(time);

    t_hr_p   = time_hr(idxp);
    T4_log_p = T4_log(idxp);
    u_pid_p  = u_pid(idxp);
    EF_p     = EF_log(idxp);

    % ================================================================
    % Tracking band
    % ================================================================
    band_pct_plot = 0.05;

    band_hi = T4_des * (1 + band_pct_plot);
    band_lo = T4_des * (1 - band_pct_plot);

    % ================================================================
    % Main figure
    % ================================================================
    fig = figure( ...
        'Color','w', ...
        'Position',[100 100 1800 1250]);

    % Readability settings
    FS_axes   = 22;
    FS_label  = 24;
    FS_title  = 25;
    FS_legend = 18;
    FS_panel  = 26;

    LW        = 2.5;
    LW_ref    = 2.2;
    MS        = 6;


    % ================================================================
    % A. Tracking
    % ================================================================
    ax1 = subplot(3,2,1);
    hold(ax1,'on');

    patch(ax1, ...
        [t_hr_p, fliplr(t_hr_p)], ...
        [band_lo*ones(size(t_hr_p)), ...
         fliplr(band_hi*ones(size(t_hr_p)))], ...
        [0.92 0.92 0.92], ...
        'EdgeColor','none', ...
        'FaceAlpha',0.5, ...
        'DisplayName','\pm5% band');

    plot(ax1, ...
        t_hr_p, ...
        T4_des*ones(size(t_hr_p)), ...
        '--k', ...
        'LineWidth',LW_ref, ...
        'DisplayName','Setpoint');

    plot(ax1, ...
        t_hr_p, ...
        T4_log_p, ...
        '-r', ...
        'LineWidth',LW, ...
        'DisplayName','T_4^{ext} (continuous)');

    % One measurement object instead of separate line and marker
    % legend entries.
    plot(ax1, ...
        t_meas_hr, ...
        T4_meas_valid, ...
        '-o', ...
        'Color',[0.85 0.50 0], ...
        'LineWidth',1.5, ...
        'MarkerSize',MS, ...
        'MarkerFaceColor',[0.25 0.25 0.25], ...
        'MarkerEdgeColor',[0.25 0.25 0.25], ...
        'DisplayName','Measured');

    xlabel(ax1, ...
        'Time (h)', ...
        'FontSize',FS_label);

    yl1 = ylabel(ax1, ...
        'T_4^{ext} (a.u.)', ...
        'FontSize',FS_label);

    title(ax1, ...
        'Tracking', ...
        'FontSize',FS_title);

    grid(ax1,'on');

    % Add vertical headroom so the trajectory is not obscured
    % by the legend.
    trackVals = [ ...
        T4_log_p(:); ...
        T4_meas_valid(:); ...
        T4_des; ...
        band_hi; ...
        band_lo];

    trackVals = trackVals(isfinite(trackVals));

    if ~isempty(trackVals)

        trackMin = min(trackVals);
        trackMax = max(trackVals);

        trackSpan = max(trackMax-trackMin,1);

        trackLower = max(0, ...
            trackMin - 0.08*trackSpan);

        trackUpper = ...
            trackMax + 0.30*trackSpan;

        ylim(ax1,[trackLower trackUpper]);
    end

    lgd1 = legend(ax1, ...
        'Location','southeast', ...
        'FontSize',FS_legend);

    lgd1.Box = 'on';


    % ================================================================
    % B. PID command
    % ================================================================
    ax2 = subplot(3,2,2);
    hold(ax2,'on');

    plot(ax2, ...
        t_hr_p, ...
        u_pid_p, ...
        '-b', ...
        'LineWidth',LW, ...
        'DisplayName','u_{PID}');

    yline(ax2, ...
        thresh, ...
        '--', ...
        'Color',[0.85 0.33 0.10], ...
        'LineWidth',LW_ref, ...
        'DisplayName','threshold');

    % Add headroom around threshold/boundaries.
    pidValid = u_pid_p(isfinite(u_pid_p));

    pidVals = [ ...
        pidValid(:); ...
        thresh];

    if ~isempty(pidVals)

        pidLo = min(pidVals);
        pidHi = max(pidVals);

        pidSpan = max( ...
            pidHi-pidLo, ...
            max(1e-6,0.05*max(abs(pidVals))));

        ylim(ax2,[ ...
            pidLo - 0.08*pidSpan, ...
            pidHi + 0.08*pidSpan]);
    end

    xlabel(ax2, ...
        'Time (h)', ...
        'FontSize',FS_label);

    yl2 = ylabel(ax2, ...
        'u_{PID} (a.u.)', ...
        'FontSize',FS_label);

    title(ax2, ...
        'PID command', ...
        'FontSize',FS_title);

    grid(ax2,'on');

    lgd2 = legend(ax2, ...
        'Location','best', ...
        'FontSize',FS_legend);

    lgd2.Box = 'on';


    % ================================================================
    % C. EF supplied
    % ================================================================
    ax3 = subplot(3,2,3);
    hold(ax3,'on');

    plot(ax3, ...
        t_hr_p, ...
        EF_p, ...
        'm', ...
        'LineWidth',LW);

    xlabel(ax3, ...
        'Time (h)', ...
        'FontSize',FS_label);

    yl3 = ylabel(ax3, ...
        'EF (avg, bursts; a.u.)', ...
        'FontSize',FS_label);

    title(ax3, ...
        'EF supplied', ...
        'FontSize',FS_title);

    grid(ax3,'on');


    % ================================================================
    % D. Per-window applied amplitude
    % ================================================================
    ax4 = subplot(3,2,4);
    hold(ax4,'on');

    stairs(ax4, ...
        sample_times(maskA)/60, ...
        A_hist(maskA), ...
        'LineWidth',LW, ...
        'DisplayName','A_{applied}');

    yline(ax4, ...
        A_max, ...
        '--r', ...
        'LineWidth',LW_ref, ...
        'DisplayName','A_{max}');

    % Add visible space above A_max so it does not coincide
    % with the upper frame.
    Avalid = A_hist(maskA);
    Afinite = Avalid(isfinite(Avalid));

    ampVals = [ ...
        Afinite(:); ...
        A_max; ...
        0];

    if ~isempty(ampVals)

        ampLo = min(ampVals);
        ampHi = max(ampVals);

        ampSpan = max( ...
            ampHi-ampLo, ...
            max(1e-6,0.05*max(abs(ampVals))));

        ylim(ax4,[ ...
            max(0,ampLo-0.08*ampSpan), ...
            ampHi+0.10*ampSpan]);
    end

    xlabel(ax4, ...
        'Time (h)', ...
        'FontSize',FS_label);

    yl4 = ylabel(ax4, ...
        'A_{applied} (a.u.)', ...
        'FontSize',FS_label);

    title(ax4, ...
        'Per-window amplitude', ...
        'FontSize',FS_title);

    grid(ax4,'on');

    lgd4 = legend(ax4, ...
        'Location','best', ...
        'FontSize',FS_legend);

    lgd4.Box = 'on';


    % ================================================================
    % E. Normalized adaptive PID gains
    % ================================================================
    ax5 = subplot(3,2,5);
    hold(ax5,'on');

    % First recorded non-NaN gains define the normalization.
    i0Kp = find(~isnan(Kp_hist),1,'first');
    i0Ki = find(~isnan(Ki_hist),1,'first');
    i0Kd = find(~isnan(Kd_hist),1,'first');

    if isempty(i0Kp) || isempty(i0Ki) || isempty(i0Kd)
        error('PID gain histories contain no valid initial values.');
    end

    Kp0 = Kp_hist(i0Kp);
    Ki0 = Ki_hist(i0Ki);
    Kd0 = Kd_hist(i0Kd);

    % Normalize each gain by its own initial value.
    Kp_rel = Kp_hist / max(abs(Kp0),eps);
    Ki_rel = Ki_hist / max(abs(Ki0),eps);
    Kd_rel = Kd_hist / max(abs(Kd0),eps);

    stairs(ax5, ...
        sample_times/60, ...
        Kp_rel, ...
        'LineWidth',LW, ...
        'DisplayName','K_p/K_{p,0}');

    stairs(ax5, ...
        sample_times/60, ...
        Ki_rel, ...
        'LineWidth',LW, ...
        'DisplayName','K_i/K_{i,0}');

    stairs(ax5, ...
        sample_times/60, ...
        Kd_rel, ...
        'LineWidth',LW, ...
        'DisplayName','K_d/K_{d,0}');

    % Reference: initial gain value = 1.
    yline(ax5, ...
        1, ...
        '--k', ...
        'LineWidth',1.3, ...
        'HandleVisibility','off');

    % Give normalized trajectories some vertical headroom.
    KpFinite = Kp_rel(isfinite(Kp_rel));
    KiFinite = Ki_rel(isfinite(Ki_rel));
    KdFinite = Kd_rel(isfinite(Kd_rel));

    gainVals = [ ...
        KpFinite(:); ...
        KiFinite(:); ...
        KdFinite(:); ...
        1];

    if ~isempty(gainVals)

        gainLo = min(gainVals);
        gainHi = max(gainVals);

        gainSpan = max( ...
            gainHi-gainLo, ...
            max(1e-6,0.05*max(abs(gainVals))));

        ylim(ax5,[ ...
            max(0,gainLo-0.08*gainSpan), ...
            gainHi+0.10*gainSpan]);
    end

    xlabel(ax5, ...
        'Time (h)', ...
        'FontSize',FS_label);

    yl5 = ylabel(ax5, ...
        'Gain / initial gain', ...
        'FontSize',FS_label);

    title(ax5, ...
        'Normalized adaptive PID gains', ...
        'FontSize',FS_title);

    grid(ax5,'on');

    lgd5 = legend(ax5, ...
        'Location','best', ...
        'FontSize',FS_legend);

    lgd5.Box = 'on';


    % ================================================================
    % F. Per-window predictive cost
    % ================================================================
    ax6 = subplot(3,2,6);
    hold(ax6,'on');

    stairs(ax6, ...
        sample_times(maskJ)/60, ...
        cost_hist(maskJ), ...
        'LineWidth',LW);

    xlabel(ax6, ...
        'Time (h)', ...
        'FontSize',FS_label);

    yl6 = ylabel(ax6, ...
        '$J_m$', ...
        'FontSize',FS_label, ...
        'Interpreter','latex');

    title(ax6, ...
        'Per-window cost', ...
        'FontSize',FS_title);

    grid(ax6,'on');

    % Keep the scientific-notation presentation used in the paper.
    finiteCost = cost_hist(maskJ);
    finiteCost = finiteCost(isfinite(finiteCost));

    if ~isempty(finiteCost) && max(abs(finiteCost)) >= 1e3
        ax6.YAxis.Exponent = 4;
    end


    % ================================================================
    % Common axis formatting
    % ================================================================
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

    % ================================================================
    % External panel labels A--F
    % ================================================================
    %
    % These are figure annotations rather than text inside the axes.
    % Therefore labels such as E do not collide with the gain legend
    % and are not clipped by the axes.
    %
    drawnow;

    add_panel_label_outside(fig,ax1,'A',FS_panel);
    add_panel_label_outside(fig,ax2,'B',FS_panel);
    add_panel_label_outside(fig,ax3,'C',FS_panel);
    add_panel_label_outside(fig,ax4,'D',FS_panel);
    add_panel_label_outside(fig,ax5,'E',FS_panel);
    add_panel_label_outside(fig,ax6,'F',FS_panel);


    % ================================================================
    % Diagnostics
    % ================================================================
    if opts.verbose

        fprintf('\n=== Tracking diagnostics ===\n');

        fprintf( ...
            'N full points           = %d\n', ...
            numel(time));

        fprintf( ...
            'N plotted points        = %d\n', ...
            numel(idxp));

        fprintf( ...
            'Max continuous T4       = %.6f\n', ...
            max(T4_log));

        fprintf( ...
            'Min continuous error    = %.6f\n', ...
            min(T4_des-T4_log));

        if any(mask_meas)

            fprintf( ...
                'Max sampled T4          = %.6f\n', ...
                max(T4_meas_valid));

            fprintf( ...
                'Min sampled error       = %.6f\n', ...
                min(T4_des-T4_meas_valid));
        end

        fprintf('\nInitial APID gains used for normalization:\n');
        fprintf('Kp0 = %.8g\n',Kp0);
        fprintf('Ki0 = %.8g\n',Ki0);
        fprintf('Kd0 = %.8g\n',Kd0);
    end


    % ================================================================
    % Optional EF zoom diagnostic
    % ================================================================
    %
    % Kept from the original plotting routine. This does not affect
    % the manuscript summary figure.
    %
    Nzoom = numel(idxp);

    i1 = max(1,round(0.15*Nzoom));
    i2 = min(Nzoom,round(0.30*Nzoom));

    fig_zoom = figure( ...
        'Color','w', ...
        'Position',[820 120 750 400]);

    axz = axes(fig_zoom);
    hold(axz,'on');

    plot(axz, ...
        t_hr_p(i1:i2), ...
        EF_p(i1:i2), ...
        'm', ...
        'LineWidth',LW);

    xlabel(axz, ...
        'Time (h)', ...
        'FontSize',FS_label);

    ylabel(axz, ...
        'EF (avg, bursts; a.u.)', ...
        'FontSize',FS_label);

    title(axz, ...
        'EF supplied (zoomed subset)', ...
        'FontSize',FS_title);

    set(axz, ...
        'FontSize',FS_axes, ...
        'LineWidth',1.4, ...
        'Box','on', ...
        'TickDir','out');

    grid(axz,'on');


    % ================================================================
    % Save manuscript figure
    % ================================================================
    if opts.do_save_pdf

        if ~exist(opts.output_dir,'dir')
            mkdir(opts.output_dir);
        end

        pdfFile = fullfile( ...
            opts.output_dir, ...
            sprintf('apid_setpoint_%g.pdf',OUT.T4_des));

        drawnow;

        exportgraphics( ...
            fig, ...
            pdfFile, ...
            'ContentType','vector', ...
            'BackgroundColor','white');

        if opts.verbose
            fprintf('\nSaved APID figure:\n%s\n',pdfFile);
        end
    end

end


% =====================================================================
% Helper: external panel label
% =====================================================================
function h = add_panel_label_outside(fig,ax,labelText,FS_panel)
%ADD_PANEL_LABEL_OUTSIDE
% Place panel label just outside the upper-left corner of the axes.
%
% Annotation uses figure-normalized coordinates, so the label is not
% clipped by the axes and does not cover plotted data.

    drawnow;

    oldUnits = ax.Units;

    ax.Units = 'normalized';
    pos = ax.Position;
    ax.Units = oldUnits;

    % Slightly left of and above the axes box.
    x = pos(1) - 0.040;
    y = pos(2) + pos(4) + 0.006;

    % Keep labels inside the exported figure canvas.
    x = max(0.005,x);
    y = min(0.965,y);

    h = annotation( ...
        fig, ...
        'textbox', ...
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