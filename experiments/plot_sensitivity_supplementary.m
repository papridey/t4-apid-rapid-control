function FILES = plot_sensitivity_supplementary(resultsMatFile, figureDir, opts)
%PLOT_SENSITIVITY_SUPPLEMENTARY
% Plot-only companion to run_sensitivity_exact_current_model.m.
%
% This function DOES NOT rerun any ODE simulations or sensitivity sweeps.
% It loads the saved OUT structure from:
%
%   sensitivity_exact_current_model.mat
%
% and regenerates Supplementary Figs. S1--S4:
%
%   Fig_S1_nominal_metrics
%   Fig_S2_delay_assumptions
%   Fig_S3_local_parameter_sensitivity
%   Fig_S4_downstream_hill_exponent
%
% Each is saved as:
%   .fig  editable MATLAB figure
%   .pdf  vector manuscript figure
%   .png  high-resolution raster copy
%
% Example:
%
% projectRoot = ...
%   '/Users/papridey/Documents/UCSC/BIO-INSYNC/Journal Paper/TA_APID-control_final';
%
% resultsMatFile = fullfile(projectRoot,'results', ...
%   'sensitivity_exact_current_model', ...
%   'sensitivity_exact_current_model.mat');
%
% figureDir = fullfile(projectRoot,'results', ...
%   'sensitivity_exact_current_model','figures');
%
% plot_sensitivity_supplementary(resultsMatFile,figureDir);
%
% You can repeatedly edit this file and rerun it without rerunning the
% sensitivity calculations.

if nargin < 1 || isempty(resultsMatFile)
    error('Please provide sensitivity_exact_current_model.mat.');
end

if exist(resultsMatFile,'file') ~= 2
    error('Results MAT file not found:\n%s',resultsMatFile);
end

if nargin < 2 || isempty(figureDir)
    figureDir = fullfile(fileparts(resultsMatFile),'figures');
end

if nargin < 3 || isempty(opts)
    opts = struct();
end

opts = plotting_defaults(opts);

if ~exist(figureDir,'dir')
    mkdir(figureDir);
end

S = load(resultsMatFile,'OUT');

if ~isfield(S,'OUT')
    error('The MAT file does not contain OUT.');
end

OUT = S.OUT;

required = {'Nominal','Local_OAT','Delay','N_shape','h_struct', ...
            'nominal_time_h','nominal_T4ext','parameter_list'};

for k = 1:numel(required)
    if ~isfield(OUT,required{k})
        error('OUT.%s is missing. Re-run the numerical sensitivity file once.', ...
            required{k});
    end
end

fprintf('\nPlotting Supplementary Figs. S1--S4 from saved results only.\n');
fprintf('Results file : %s\n',resultsMatFile);
fprintf('Figure folder: %s\n\n',figureDir);

FILES = struct();
FILES.S1 = make_S1_nominal_metrics(OUT,figureDir,opts);
FILES.S2 = make_S2_delay_sensitivity(OUT,figureDir,opts);
FILES.S3 = make_S3_local_parameter_sensitivity(OUT,figureDir,opts);
FILES.S4 = make_S4_hill_exponent_sensitivity(OUT,figureDir,opts);

fprintf('\nGenerated:\n');
fprintf('  S1: %s\n',FILES.S1.pdf);
fprintf('  S2: %s\n',FILES.S2.pdf);
fprintf('  S3: %s\n',FILES.S3.pdf);
fprintf('  S4: %s\n',FILES.S4.pdf);

end


%% ========================================================================
% Plot defaults
% =========================================================================
function opts = plotting_defaults(opts)

opts = setdef(opts,'figure_visible',true);
opts = setdef(opts,'save_fig',true);
opts = setdef(opts,'save_pdf',true);
opts = setdef(opts,'save_png',true);
opts = setdef(opts,'png_resolution',600);

% S1 ranges
opts = setdef(opts,'S1_transient_hours',25);
opts = setdef(opts,'S1_tail_hours',10);

% Global publication font sizes
opts = setdef(opts,'FS_axes',24);
opts = setdef(opts,'FS_label',27);
opts = setdef(opts,'FS_title',26);
opts = setdef(opts,'FS_panel',24);
opts = setdef(opts,'FS_legend',18);
opts = setdef(opts,'FS_annotation',20);

opts = setdef(opts,'LW',3.0);
opts = setdef(opts,'LW_ref',2.0);
opts = setdef(opts,'MS',8);

end


%% ========================================================================
% S1. Metric definitions
% =========================================================================
function files = make_S1_nominal_metrics(OUT,figureDir,opts)

t = OUT.nominal_time_h(:);
y = OUT.nominal_T4ext(:);
N = OUT.Nominal;

basal  = N.T4_basal(1);
ss     = N.T4_ss(1);
t10    = N.t10_h(1);
t90    = N.t90_h(1);
riseh  = N.rise_h(1);
rip    = N.ripple(1);
rippct = N.ripple_pct(1);
period = N.ripple_period_min(1);

fig = figure('Color','w','Visible',onoff(opts.figure_visible), ...
    'Units','pixels','Position',[80 80 2300 900]);

tl = tiledlayout(fig,1,2,'TileSpacing','loose','Padding','loose');

% ------------------------------------------------------------------------
% A. Transient
% -------------------------------------------------------------------------
ax1 = nexttile(tl,1);
hold(ax1,'on');

mask = t <= opts.S1_transient_hours;

hTraj = plot(ax1,t(mask),y(mask),'-', ...
    'LineWidth',opts.LW,'DisplayName','T_4^{ext}(t)');

hBasal = yline(ax1,basal,'--', ...
    'LineWidth',opts.LW_ref,'DisplayName','Basal level');

hSS = yline(ax1,ss,'--', ...
    'LineWidth',opts.LW_ref,'DisplayName','Steady state');

if isfinite(t10)
    y10 = basal + 0.10*(ss-basal);

    xline(ax1,t10,':', ...
        'LineWidth',1.5,'HandleVisibility','off');

    plot(ax1,t10,y10,'o', ...
        'MarkerSize',opts.MS, ...
        'LineWidth',2, ...
        'HandleVisibility','off');

    text(ax1,t10,y10,sprintf('  t_{10}=%.2f h',t10), ...
        'FontSize',opts.FS_annotation, ...
        'VerticalAlignment','bottom', ...
        'HandleVisibility','off');
end

if isfinite(t90)
    y90 = basal + 0.90*(ss-basal);

    xline(ax1,t90,':', ...
        'LineWidth',1.5,'HandleVisibility','off');

    plot(ax1,t90,y90,'s', ...
        'MarkerSize',opts.MS, ...
        'LineWidth',2, ...
        'HandleVisibility','off');

    text(ax1,t90,y90,sprintf('  t_{90}=%.2f h',t90), ...
        'FontSize',opts.FS_annotation, ...
        'VerticalAlignment','bottom', ...
        'HandleVisibility','off');
end

text(ax1,0.51*opts.S1_transient_hours, ...
    basal+0.075*(ss-basal), ...
    sprintf('rise time = %.2f h',riseh), ...
    'FontSize',opts.FS_annotation, ...
    'FontWeight','bold', ...
    'HandleVisibility','off');

xlabel(ax1,'Time (h)');
ylabel(ax1,'T_4^{ext} (a.u.)');
title(ax1,'Transient response and rise-time definition');
grid(ax1,'on');
xlim(ax1,[0 opts.S1_transient_hours]);
add_plot_headroom(ax1,y(mask),0.12);

legend(ax1,[hTraj hBasal hSS], ...
    {'T_4^{ext}(t)','Basal level','Steady state'}, ...
    'Location','southeast', ...
    'FontSize',opts.FS_legend, ...
    'Box','on');

% ------------------------------------------------------------------------
% B. Late-time ripple
% -------------------------------------------------------------------------
ax2 = nexttile(tl,2);
hold(ax2,'on');

t0 = max(t(1),t(end)-opts.S1_tail_hours);
mask = t >= t0;
ytail = y(mask);

hTail = plot(ax2,t(mask),ytail,'-', ...
    'LineWidth',opts.LW, ...
    'DisplayName','T_4^{ext}(t)');

hMean = yline(ax2,ss,'--', ...
    'LineWidth',opts.LW_ref, ...
    'DisplayName','Steady-state mean');

ymin = min(ytail);
ymax = max(ytail);

yline(ax2,ymin,':', ...
    'LineWidth',1.3,'HandleVisibility','off');

yline(ax2,ymax,':', ...
    'LineWidth',1.3,'HandleVisibility','off');

xText = t0 + 0.035*(t(end)-t0);

text(ax2,xText,ymax, ...
    sprintf('\\Delta T_4^{pp}=%.3f a.u. (%.2f%%)',rip,rippct), ...
    'FontSize',opts.FS_annotation, ...
    'VerticalAlignment','bottom', ...
    'HandleVisibility','off');

if isfinite(period)
    perh = period/60;

    xb1 = t(end)-2.2*perh;
    xb2 = xb1+perh;
    yr  = ymin + 0.13*(ymax-ymin);

    plot(ax2,[xb1 xb2],[yr yr],'-', ...
        'LineWidth',2.1,'HandleVisibility','off');

    plot(ax2,[xb1 xb1], ...
        [yr-0.035*(ymax-ymin),yr+0.035*(ymax-ymin)], ...
        '-','LineWidth',1.6,'HandleVisibility','off');

    plot(ax2,[xb2 xb2], ...
        [yr-0.035*(ymax-ymin),yr+0.035*(ymax-ymin)], ...
        '-','LineWidth',1.6,'HandleVisibility','off');

    text(ax2,mean([xb1 xb2]),yr, ...
        sprintf('T_s \\approx %.0f min',period), ...
        'HorizontalAlignment','center', ...
        'VerticalAlignment','bottom', ...
        'FontSize',opts.FS_annotation, ...
        'HandleVisibility','off');
end

xlabel(ax2,'Time (h)');
ylabel(ax2,'T_4^{ext} (a.u.)');
title(ax2,'Steady-state ripple amplitude and period');
grid(ax2,'on');
xlim(ax2,[t0 t(end)]);
add_plot_headroom(ax2,ytail,0.18);

legend(ax2,[hTail hMean], ...
    {'T_4^{ext}(t)','Steady-state mean'}, ...
    'Location','northeast', ...
    'FontSize',opts.FS_legend, ...
    'Box','on');

format_axes_pub([ax1 ax2],opts);

add_panel_letter(fig,ax1,'A',opts.FS_panel);
add_panel_letter(fig,ax2,'B',opts.FS_panel);

base = fullfile(figureDir,'Fig_S1_nominal_metrics');
files = save_pub_figure(fig,base,opts);

end


%% ========================================================================
% S2. Delay sensitivity
% =========================================================================
function files = make_S2_delay_sensitivity(OUT,figureDir,opts)

Ns = sortrows(OUT.N_shape,'N');
Ds = sortrows(OUT.Delay,'mean_delay_h');

fig = figure('Color','w','Visible',onoff(opts.figure_visible), ...
    'Units','pixels','Position',[60 60 2400 1450]);

% Deliberately no overall sgtitle/tiledlayout title:
% the row meaning is explained in the manuscript caption, avoiding collision
% between panel letters and a long figure-level title.
tl = tiledlayout(fig,2,3,'TileSpacing','loose','Padding','loose');

ax1 = nexttile(tl,1);
plot(ax1,Ns.N,Ns.T4_ss,'-o', ...
    'LineWidth',opts.LW,'MarkerSize',opts.MS);
xlabel(ax1,'Delay-chain order N');
ylabel(ax1,'Steady-state T_4^{ext} (a.u.)');
title(ax1,'Steady-state output');
grid(ax1,'on');

ax2 = nexttile(tl,2);
plot(ax2,Ns.N,Ns.rise_h,'-o', ...
    'LineWidth',opts.LW,'MarkerSize',opts.MS);
xlabel(ax2,'Delay-chain order N');
ylabel(ax2,'Rise time (h)');
title(ax2,'Rise time');
grid(ax2,'on');

ax3 = nexttile(tl,3);
semilogy(ax3,Ns.N,max(Ns.ripple,eps),'-o', ...
    'LineWidth',opts.LW,'MarkerSize',opts.MS);
xlabel(ax3,'Delay-chain order N');
ylabel(ax3,'Ripple amplitude (a.u.)');
title(ax3,'Ripple amplitude');
grid(ax3,'on');

ax4 = nexttile(tl,4);
plot(ax4,Ds.mean_delay_h,Ds.T4_ss,'-o', ...
    'LineWidth',opts.LW,'MarkerSize',opts.MS);
xlabel(ax4,'Mean delay N/a (h)');
ylabel(ax4,'Steady-state T_4^{ext} (a.u.)');
title(ax4,'Steady-state output');
grid(ax4,'on');

ax5 = nexttile(tl,5);
plot(ax5,Ds.mean_delay_h,Ds.rise_h,'-o', ...
    'LineWidth',opts.LW,'MarkerSize',opts.MS);
xlabel(ax5,'Mean delay N/a (h)');
ylabel(ax5,'Rise time (h)');
title(ax5,'Rise time');
grid(ax5,'on');

ax6 = nexttile(tl,6);
semilogy(ax6,Ds.mean_delay_h,max(Ds.ripple,eps),'-o', ...
    'LineWidth',opts.LW,'MarkerSize',opts.MS);
xlabel(ax6,'Mean delay N/a (h)');
ylabel(ax6,'Ripple amplitude (a.u.)');
title(ax6,'Ripple amplitude');
grid(ax6,'on');

axesList = [ax1 ax2 ax3 ax4 ax5 ax6];
format_axes_pub(axesList,opts);

letters = {'A','B','C','D','E','F'};
for k = 1:numel(axesList)
    add_panel_letter(fig,axesList(k),letters{k},opts.FS_panel);
end

base = fullfile(figureDir,'Fig_S2_delay_assumptions');
files = save_pub_figure(fig,base,opts);

end


%% ========================================================================
% S3. Local OAT curves
% =========================================================================
function files = make_S3_local_parameter_sensitivity(OUT,figureDir,opts)

T = OUT.Local_OAT;
plist = OUT.parameter_list;

Mss0  = OUT.Nominal.T4_ss(1);
Mr0   = OUT.Nominal.rise_h(1);
Mrip0 = OUT.Nominal.ripple(1);

fig = figure('Color','w','Visible',onoff(opts.figure_visible), ...
    'Units','pixels','Position',[40 70 2600 1100]);

tl = tiledlayout(fig,1,3,'TileSpacing','loose','Padding','loose');

ax1 = nexttile(tl,1); hold(ax1,'on');
ax2 = nexttile(tl,2); hold(ax2,'on');
ax3 = nexttile(tl,3); hold(ax3,'on');

groups = unique(string({plist.group}),'stable');
groupColors = lines(numel(groups));

markers = {'o','s','^','d','v','>','<','p','h','x','+','*'};
lineStyles = {'-','--','-.',':'};

legendHandles = gobjects(numel(plist),1);
legendLabels = strings(numel(plist),1);

localMultipliers = sort(unique(T.multiplier(:)))';

for ip = 1:numel(plist)

    field = string(plist(ip).code_field);
    pname = string(plist(ip).manuscript_name);
    grp   = string(plist(ip).group);

    Ti = T(T.code_field==field,:);
    Ti = sortrows(Ti,'multiplier');

    gi = find(groups==grp,1,'first');

    c  = groupColors(gi,:);
    mk = markers{1+mod(ip-1,numel(markers))};
    ls = lineStyles{1+mod(ip-1,numel(lineStyles))};

    x  = Ti.multiplier;
    y1 = Ti.T4_ss/Mss0;
    y2 = Ti.rise_h/Mr0;
    y3 = Ti.ripple/Mrip0;

    legendHandles(ip) = plot(ax1,x,y1, ...
        'LineStyle',ls,'Marker',mk,'Color',c, ...
        'LineWidth',2.2,'MarkerSize',7.5, ...
        'DisplayName',char(pname));

    plot(ax2,x,y2, ...
        'LineStyle',ls,'Marker',mk,'Color',c, ...
        'LineWidth',2.2,'MarkerSize',7.5, ...
        'HandleVisibility','off');

    plot(ax3,x,y3, ...
        'LineStyle',ls,'Marker',mk,'Color',c, ...
        'LineWidth',2.2,'MarkerSize',7.5, ...
        'HandleVisibility','off');

    legendLabels(ip) = pname;
end

for ax = [ax1 ax2 ax3]

    xline(ax,1,':','LineWidth',1.3,'HandleVisibility','off');
    yline(ax,1,'--','LineWidth',1.3,'HandleVisibility','off');

    xlim(ax,[min(localMultipliers) max(localMultipliers)]);
    xticks(ax,localMultipliers);
    grid(ax,'on');
end

xlabel(ax1,'Parameter multiplier \theta_i/\theta_{i,0}');
ylabel(ax1,'Normalized steady-state T_4^{ext}');
title(ax1,'Steady-state output');

xlabel(ax2,'Parameter multiplier \theta_i/\theta_{i,0}');
ylabel(ax2,'Normalized rise time');
title(ax2,'Rise time (10--90%)');

xlabel(ax3,'Parameter multiplier \theta_i/\theta_{i,0}');
ylabel(ax3,'Normalized ripple amplitude');
title(ax3,'Ripple amplitude');

format_axes_pub([ax1 ax2 ax3],opts);

lgd = legend(ax1,legendHandles,cellstr(legendLabels), ...
    'Interpreter','tex', ...
    'NumColumns',4, ...
    'FontSize',15, ...
    'Box','on');

lgd.Layout.Tile = 'south';

add_panel_letter(fig,ax1,'A',opts.FS_panel);
add_panel_letter(fig,ax2,'B',opts.FS_panel);
add_panel_letter(fig,ax3,'C',opts.FS_panel);

base = fullfile(figureDir,'Fig_S3_local_parameter_sensitivity');
files = save_pub_figure(fig,base,opts);

end


%% ========================================================================
% S4. Structural h sensitivity
% =========================================================================
function files = make_S4_hill_exponent_sensitivity(OUT,figureDir,opts)

H = sortrows(OUT.h_struct,'h');

fig = figure('Color','w','Visible',onoff(opts.figure_visible), ...
    'Units','pixels','Position',[70 70 2200 1500]);

tl = tiledlayout(fig,2,2,'TileSpacing','loose','Padding','loose');

ax1 = nexttile(tl,1);
plot(ax1,H.h,H.T4_ss,'-o', ...
    'LineWidth',opts.LW,'MarkerSize',opts.MS);
xline(ax1,3,'--','LineWidth',1.5,'HandleVisibility','off');
xlabel(ax1,'Downstream Hill exponent h');
ylabel(ax1,'Steady-state T_4^{ext} (a.u.)');
title(ax1,'Stimulated steady-state level');
grid(ax1,'on');

ax2 = nexttile(tl,2);
plot(ax2,H.h,H.T4_basal,'-o', ...
    'LineWidth',opts.LW,'MarkerSize',opts.MS);
xline(ax2,3,'--','LineWidth',1.5,'HandleVisibility','off');
xlabel(ax2,'Downstream Hill exponent h');
ylabel(ax2,'Basal T_4^{ext} (a.u.)');
title(ax2,'Basal level');
grid(ax2,'on');

ax3 = nexttile(tl,3);
plot(ax3,H.h,H.rise_h,'-o', ...
    'LineWidth',opts.LW,'MarkerSize',opts.MS);
xline(ax3,3,'--','LineWidth',1.5,'HandleVisibility','off');
xlabel(ax3,'Downstream Hill exponent h');
ylabel(ax3,'Rise time (h)');
title(ax3,'Rise time');
grid(ax3,'on');

ax4 = nexttile(tl,4);
plot(ax4,H.h,H.ripple,'-o', ...
    'LineWidth',opts.LW,'MarkerSize',opts.MS);
xline(ax4,3,'--','LineWidth',1.5,'HandleVisibility','off');
xlabel(ax4,'Downstream Hill exponent h');
ylabel(ax4,'Ripple amplitude (a.u.)');
title(ax4,'Ripple amplitude');
grid(ax4,'on');

format_axes_pub([ax1 ax2 ax3 ax4],opts);

add_panel_letter(fig,ax1,'A',opts.FS_panel);
add_panel_letter(fig,ax2,'B',opts.FS_panel);
add_panel_letter(fig,ax3,'C',opts.FS_panel);
add_panel_letter(fig,ax4,'D',opts.FS_panel);

base = fullfile(figureDir,'Fig_S4_downstream_hill_exponent');
files = save_pub_figure(fig,base,opts);

end


%% ========================================================================
% Common formatting
% =========================================================================
function format_axes_pub(axs,opts)

for k = 1:numel(axs)

    ax = axs(k);

    ax.FontSize  = opts.FS_axes;
    ax.LineWidth = 1.6;
    ax.Box       = 'on';
    ax.TickDir   = 'out';

    ax.XLabel.FontSize = opts.FS_label;
    ax.YLabel.FontSize = opts.FS_label;

    ax.Title.FontSize   = opts.FS_title;
    ax.Title.FontWeight = 'bold';
end

end


function add_panel_letter(fig,ax,labelText,FS)
% Place panel labels outside upper-left corner, but lower than before.
% This avoids collision with long figure-level titles.

drawnow;

oldUnits = ax.Units;
ax.Units = 'normalized';
pos = ax.Position;
ax.Units = oldUnits;

x = max(0.004,pos(1)-0.037);
y = min(0.952,pos(2)+pos(4)-0.001);

annotation(fig,'textbox',[x y 0.033 0.033], ...
    'String',labelText, ...
    'FontSize',FS, ...
    'FontWeight','bold', ...
    'HorizontalAlignment','left', ...
    'VerticalAlignment','bottom', ...
    'EdgeColor','none', ...
    'BackgroundColor','none', ...
    'Margin',0, ...
    'FitBoxToText','on');

end


function add_plot_headroom(ax,values,frac)

values = values(isfinite(values));

if isempty(values)
    return;
end

lo = min(values);
hi = max(values);
span = hi-lo;

if span <= eps
    span = max(abs(hi),1);
end

ylim(ax,[lo-0.08*span,hi+frac*span]);

end


function files = save_pub_figure(fig,base,opts)

files = struct();
files.fig = [base '.fig'];
files.pdf = [base '.pdf'];
files.png = [base '.png'];

drawnow;

if opts.save_fig
    savefig(fig,files.fig);
end

if opts.save_pdf
    exportgraphics(fig,files.pdf, ...
        'ContentType','vector', ...
        'BackgroundColor','white');
end

if opts.save_png
    exportgraphics(fig,files.png, ...
        'Resolution',opts.png_resolution, ...
        'BackgroundColor','white');
end

close(fig);

end


function s = onoff(flag)

if flag
    s = 'on';
else
    s = 'off';
end

end


function s = setdef(s,f,v)

if ~isfield(s,f) || isempty(s.(f))
    s.(f) = v;
end

end
