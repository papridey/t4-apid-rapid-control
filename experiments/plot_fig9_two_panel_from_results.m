function plot_fig9_two_panel_from_results(resultsFile, outputDir)
%PLOT_FIG9_TWO_PANEL_FROM_RESULTS
% Replot Fig. 9 using ONLY:
%   A. measured-output tracking
%   B. commanded EF input
%
% The function reads the saved RESULTS structure produced by
% compare_optimized_fixed_pid_vs_apid.m, so no controller simulations
% are rerun. It saves:
%   optimized_fixed_vs_apid_two_panel.fig   editable MATLAB figure
%   optimized_fixed_vs_apid_two_panel.pdf   vector PDF for the paper
%   optimized_fixed_vs_apid_two_panel.png   600-dpi raster copy
%
% Usage from the project root:
%
%   resultsFile = fullfile(projectRoot,'results', ...
%       'optimized_fixed_vs_apid_T4_25_revised_fig9', ...
%       'optimized_fixed_vs_apid_results.mat');
%
%   outputDir = fullfile(projectRoot,'results', ...
%       'optimized_fixed_vs_apid_T4_25_revised_fig9');
%
%   plot_fig9_two_panel_from_results(resultsFile,outputDir);
%
% If outputDir is omitted, the figure is written beside resultsFile.

    if nargin < 1 || isempty(resultsFile)
        error('Please provide the path to optimized_fixed_vs_apid_results.mat.');
    end

    if nargin < 2 || isempty(outputDir)
        outputDir = fileparts(resultsFile);
    end

    if exist(resultsFile,'file') ~= 2
        error('Results file not found:\n%s',resultsFile);
    end

    if ~exist(outputDir,'dir')
        mkdir(outputDir);
    end

    S = load(resultsFile,'RESULTS');
    if ~isfield(S,'RESULTS')
        error('The MAT file does not contain a RESULTS structure.');
    end

    R = S.RESULTS;

    required = {'representative_fixed','representative_apid','T4_des'};
    for k = 1:numel(required)
        if ~isfield(R,required{k})
            error('RESULTS.%s is missing.',required{k});
        end
    end

    F = R.representative_fixed;
    A = R.representative_apid;
    T4_des = R.T4_des;

    % Validate fields used by the two panels.
    mustHaveF = {'t_meas_hr','y_meas','t_ef_hr','ef'};
    mustHaveA = {'t_meas_hr','y_meas','t_ef_hr','ef'};
    assert_fields(F,mustHaveF,'RESULTS.representative_fixed');
    assert_fields(A,mustHaveA,'RESULTS.representative_apid');

    % -------------------------------------------------------------
    % Figure and common style
    % -------------------------------------------------------------
    fig = figure( ...
        'Color','w', ...
        'Units','pixels', ...
        'Position',[80 80 2400 1200]);

    tl = tiledlayout(fig,1,2, ...
        'TileSpacing','loose', ...
        'Padding','loose');

    LW       = 3.2;
    LW_ref   = 2.6;
    MS       = 9;
    FS_axes  = 28;
    FS_label = 30;
    FS_title = 30;
    FS_legend = 26;
    FS_panel = 32;

    % =============================================================
    % A. Measured-output tracking
    % =============================================================
    ax1 = nexttile(tl,1);
    hold(ax1,'on');

    hFixed = plot(ax1,F.t_meas_hr,F.y_meas, ...
        '-o', ...
        'LineWidth',LW, ...
        'MarkerSize',MS, ...
        'MarkerIndices',marker_indices(numel(F.t_meas_hr),18), ...
        'DisplayName','Offline-tuned fixed PID');

    hAPID = plot(ax1,A.t_meas_hr,A.y_meas, ...
        '-s', ...
        'LineWidth',LW, ...
        'MarkerSize',MS, ...
        'MarkerIndices',marker_indices(numel(A.t_meas_hr),18), ...
        'DisplayName','APID');

    hSet = yline(ax1,T4_des,'--k', ...
        'LineWidth',LW_ref, ...
        'DisplayName','Setpoint');

    xlabel(ax1,'Time (h)');
    ylabel(ax1,'Measured T_4^{ext} (a.u.)');
    title(ax1,'Measured tracking');
    grid(ax1,'on');

    add_y_headroom(ax1,[F.y_meas(:); A.y_meas(:); T4_des],0.15);

    % =============================================================
    % B. Commanded EF input
    % (this is panel C of the old four-panel Fig. 9)
    % =============================================================
    ax2 = nexttile(tl,2);
    hold(ax2,'on');

    stairs(ax2,F.t_ef_hr,F.ef, ...
        '-', ...
        'LineWidth',LW, ...
        'DisplayName','Offline-tuned fixed PID');

    stairs(ax2,A.t_ef_hr,A.ef, ...
        '-', ...
        'LineWidth',LW, ...
        'DisplayName','APID');

    xlabel(ax2,'Time (h)');
    ylabel(ax2,'Commanded EF input (a.u.)');
    title(ax2,'EF command');
    grid(ax2,'on');

    add_y_headroom(ax2,[F.ef(:); A.ef(:)],0.12);

    % -------------------------------------------------------------
    % Common formatting
    % -------------------------------------------------------------
    axesList = [ax1 ax2];
    for k = 1:numel(axesList)
        ax = axesList(k);
        ax.FontSize = FS_axes;
        ax.LineWidth = 1.8;
        ax.Box = 'on';
        ax.TickDir = 'out';
        ax.XLabel.FontSize = FS_label;
        ax.YLabel.FontSize = FS_label;
        ax.Title.FontSize = FS_title;
        ax.Title.FontWeight = 'bold';
    end

    % One common legend below both panels.
    lgd = legend(ax1,[hFixed hAPID hSet], ...
        {'Offline-tuned fixed PID','APID','Setpoint'}, ...
        'Orientation','horizontal', ...
        'FontSize',FS_legend, ...
        'Box','on');
    lgd.Layout.Tile = 'south';

    % Figure-level panel labels so they are not clipped by the axes.
    drawnow;
    add_panel_label_outside(fig,ax1,'A',FS_panel);
    add_panel_label_outside(fig,ax2,'B',FS_panel);
    drawnow;

    % -------------------------------------------------------------
    % Save editable .fig + vector PDF + high-resolution PNG
    % -------------------------------------------------------------
    baseName = fullfile(outputDir,'optimized_fixed_vs_apid_two_panel');

    savefig(fig,[baseName '.fig']);

    exportgraphics(fig,[baseName '.pdf'], ...
        'ContentType','vector', ...
        'BackgroundColor','white');

    exportgraphics(fig,[baseName '.png'], ...
        'Resolution',600, ...
        'BackgroundColor','white');

    fprintf('\nFig. 9 two-panel files saved to:\n');
    fprintf('  %s.fig\n',baseName);
    fprintf('  %s.pdf\n',baseName);
    fprintf('  %s.png\n',baseName);
end

% =====================================================================
% Helpers
% =====================================================================
function assert_fields(S,fields,structName)
    for k = 1:numel(fields)
        if ~isfield(S,fields{k})
            error('%s.%s is missing.',structName,fields{k});
        end
    end
end

function add_y_headroom(ax,values,fracTop)
    values = values(isfinite(values));
    if isempty(values)
        return;
    end

    ymin = min(0,min(values));
    ymax = max(values);
    span = ymax-ymin;

    if span <= 0
        span = max(abs(ymax),1);
    end

    ylim(ax,[ymin-0.03*span, ymax+fracTop*span]);
end

function idx = marker_indices(n,maxMarkers)
    if n <= 0
        idx = [];
    elseif n <= maxMarkers
        idx = 1:n;
    else
        idx = unique(round(linspace(1,n,maxMarkers)));
    end
end

function h = add_panel_label_outside(fig,ax,labelText,FS_panel)
    drawnow;

    oldUnits = ax.Units;
    ax.Units = 'normalized';
    pos = ax.Position;
    ax.Units = oldUnits;

    % Keep labels just outside the upper-left corner of each axes while
    % remaining safely inside the export canvas.
    x = max(0.008,pos(1)-0.035);
    y = min(0.965,pos(2)+pos(4)+0.006);

    h = annotation(fig,'textbox', ...
        [x y 0.04 0.04], ...
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
