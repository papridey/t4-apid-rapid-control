function plot_fig9_one_panel_from_results(resultsFile, outputDir)
%PLOT_FIG9_ONE_PANEL_FROM_RESULTS
%
% Main-paper Fig. 9:
%   measured-output tracking only
%
% Compares:
%   1. Offline-tuned fixed PID
%   2. APID
%   3. Desired setpoint
%
% The function reads the saved RESULTS structure produced by
% compare_optimized_fixed_pid_vs_apid.m.
%
% NO controller simulations are rerun.
%
% Files saved:
%   optimized_fixed_vs_apid_measured_tracking.fig
%   optimized_fixed_vs_apid_measured_tracking.pdf
%   optimized_fixed_vs_apid_measured_tracking.png
%
% Example:
%
% projectRoot = ...
%   '/Users/papridey/Documents/UCSC/BIO-INSYNC/Journal Paper/TA_APID-control_final';
%
% resultsFile = fullfile(projectRoot,'results', ...
%     'optimized_fixed_vs_apid_T4_25_revised_fig9', ...
%     'optimized_fixed_vs_apid_results.mat');
%
% outputDir = fullfile(projectRoot,'results', ...
%     'optimized_fixed_vs_apid_T4_25_revised_fig9');
%
% plot_fig9_one_panel_from_results(resultsFile,outputDir);


%% ============================================================
% Input checks
% =============================================================

if nargin < 1 || isempty(resultsFile)
    error(['Please provide the path to ' ...
           'optimized_fixed_vs_apid_results.mat.']);
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


%% ============================================================
% Load saved comparison
% =============================================================

S = load(resultsFile,'RESULTS');

if ~isfield(S,'RESULTS')
    error('The MAT file does not contain a RESULTS structure.');
end

R = S.RESULTS;

required = { ...
    'representative_fixed', ...
    'representative_apid', ...
    'T4_des'};

for k = 1:numel(required)
    if ~isfield(R,required{k})
        error('RESULTS.%s is missing.',required{k});
    end
end

F = R.representative_fixed;
A = R.representative_apid;
T4_des = R.T4_des;


%% ============================================================
% Validate measured-output data
% =============================================================

mustHave = {'t_meas_hr','y_meas'};

assert_fields(F,mustHave, ...
    'RESULTS.representative_fixed');

assert_fields(A,mustHave, ...
    'RESULTS.representative_apid');


%% ============================================================
% Figure style
% =============================================================

LW        = 3.2;
LW_ref    = 2.6;
MS        = 9;

FS_axes   = 28;
FS_label  = 29;
FS_title  = 29;
FS_legend = 28;


%% ============================================================
% Create single-panel figure
% =============================================================

fig = figure( ...
    'Color','w', ...
    'Units','pixels', ...
    'Position',[100 100 1500 950]);

ax = axes(fig);

hold(ax,'on');


%% ============================================================
% Offline-tuned fixed PID
% =============================================================

hFixed = plot(ax, ...
    F.t_meas_hr, ...
    F.y_meas, ...
    '-o', ...
    'LineWidth',LW, ...
    'MarkerSize',MS, ...
    'MarkerIndices', ...
        marker_indices(numel(F.t_meas_hr),18), ...
    'DisplayName','Offline-tuned fixed PID');


%% ============================================================
% APID
% =============================================================

hAPID = plot(ax, ...
    A.t_meas_hr, ...
    A.y_meas, ...
    '-s', ...
    'LineWidth',LW, ...
    'MarkerSize',MS, ...
    'MarkerIndices', ...
        marker_indices(numel(A.t_meas_hr),18), ...
    'DisplayName','APID');


%% ============================================================
% Desired setpoint
% =============================================================

hSet = yline(ax, ...
    T4_des, ...
    '--k', ...
    'LineWidth',LW_ref, ...
    'DisplayName','Setpoint');


%% ============================================================
% Labels and formatting
% =============================================================

xlabel(ax,'Time (h)');

ylabel(ax, ...
    'Measured T_4^{ext} (a.u.)');

title(ax,'Measured tracking');

grid(ax,'on');

ax.FontSize  = FS_axes;
ax.LineWidth = 1.8;
ax.Box       = 'on';
ax.TickDir   = 'out';

ax.XLabel.FontSize = FS_label;
ax.YLabel.FontSize = FS_label;

ax.Title.FontSize   = FS_title;
ax.Title.FontWeight = 'bold';

% Keep the full 80-h interval visible.
xlim(ax,[0 max([F.t_meas_hr(:); A.t_meas_hr(:)])]);

% Add modest vertical headroom.
add_y_headroom( ...
    ax, ...
    [F.y_meas(:); A.y_meas(:); T4_des], ...
    0.12);


%% ============================================================
% Legend
% =============================================================

lgd = legend(ax, ...
    [hFixed hAPID hSet], ...
    {'Offline-tuned fixed PID', ...
     'APID', ...
     'Setpoint'}, ...
    'Orientation','horizontal', ...
    'Location','northoutside', ...
    'FontSize',FS_legend, ...
    'Box','on');

drawnow;


%% ============================================================
% Save files
% =============================================================

baseName = fullfile( ...
    outputDir, ...
    'optimized_fixed_vs_apid_measured_tracking');


% Editable MATLAB figure
savefig(fig,[baseName '.fig']);


% Vector PDF for manuscript
exportgraphics(fig, ...
    [baseName '.pdf'], ...
    'ContentType','vector', ...
    'BackgroundColor','white');


% High-resolution PNG
exportgraphics(fig, ...
    [baseName '.png'], ...
    'Resolution',600, ...
    'BackgroundColor','white');


fprintf('\nMain-paper Fig. 9 saved as:\n');
fprintf('  %s.fig\n',baseName);
fprintf('  %s.pdf\n',baseName);
fprintf('  %s.png\n',baseName);

end


%% =====================================================================
% Helper: verify required fields
% ======================================================================

function assert_fields(S,fields,structName)

for k = 1:numel(fields)

    if ~isfield(S,fields{k})
        error('%s.%s is missing.', ...
            structName,fields{k});
    end

end

end


%% =====================================================================
% Helper: add vertical plotting headroom
% ======================================================================

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

ylim(ax,[ ...
    ymin-0.03*span, ...
    ymax+fracTop*span]);

end


%% =====================================================================
% Helper: limit marker density
% ======================================================================

function idx = marker_indices(n,maxMarkers)

if n <= 0

    idx = [];

elseif n <= maxMarkers

    idx = 1:n;

else

    idx = unique( ...
        round(linspace(1,n,maxMarkers)));

end

end