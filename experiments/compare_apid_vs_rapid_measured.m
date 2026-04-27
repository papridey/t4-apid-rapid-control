clear; clc; close all;

% ------------------------------------------------------------
% User settings
% ------------------------------------------------------------
T4_des = 25;

opts_apid = struct( ...
    'dt_seconds', 0.1, ...
    'sim_hours', 80, ...
    'make_plots', false, ...
    'verbose', true, ...
    'do_save_pdf', false);

opts_rapid = struct( ...
    'dt_seconds', 0.1, ...
    'sim_hours', 80, ...
    'do_plots', false, ...
    'verbose', true, ...
    'do_save_pdf', false, ...
    'rng_seed', 1, ...
    'robust_M', 5, ...
    'robust_lambda_cvar', 0.6, ...
    'robust_cvar_q', 0.80, ...
    'param_scale_default', 0.10, ...
    'kappa_scale', 0.10, ...
    'K2_scale', 0.10, ...
    'd4_bias', 0.01, ...
    'd4_amp', 0.015, ...
    'act_gain_scale', 0.10, ...
    'delay_seconds', 5, ...
    'jitter_seconds', 2);

% Bands for plotting/metrics
band_pct_plot    = 0.30;   % shown on plot
band_pct_metrics = 0.30;   % settle + in-band metric
band_pct_control = 0.10;   % tighter in-band metric also reported

% ------------------------------------------------------------
% Run controllers
% ------------------------------------------------------------
fprintf('Starting APID...\n'); drawnow;
OUT_apid = run_apid_bandlock_smooth(T4_des, opts_apid);

fprintf('Finished APID. Starting RAPID...\n'); drawnow;
OUT_rapid = run_rapid_clean5(T4_des, opts_rapid);

fprintf('Finished RAPID. Building measured-T4 comparison plot...\n'); drawnow;

% ------------------------------------------------------------
% Extract measured signals
% ------------------------------------------------------------
mask_apid = ~isnan(OUT_apid.t_meas) & ~isnan(OUT_apid.T4_meas);
t_apid_min = OUT_apid.t_meas(mask_apid);
t_apid_hr  = t_apid_min / 60;
y_apid_meas = OUT_apid.T4_meas(mask_apid);

mask_rapid = ~isnan(OUT_rapid.t_meas) & ~isnan(OUT_rapid.T4_meas_used);
t_rapid_min = OUT_rapid.t_meas(mask_rapid);
t_rapid_hr  = t_rapid_min / 60;
y_rapid_meas = OUT_rapid.T4_meas_used(mask_rapid);

% ------------------------------------------------------------
% Metrics
% ------------------------------------------------------------
M_apid  = compute_measured_metrics(t_apid_min,  y_apid_meas,  T4_des, band_pct_metrics, band_pct_control);
M_rapid = compute_measured_metrics(t_rapid_min, y_rapid_meas, T4_des, band_pct_metrics, band_pct_control);

% ------------------------------------------------------------
% Plot measured T4 comparison
% ------------------------------------------------------------
fig = figure('Color','w','Position',[120 120 1500 700]);
ax = axes(fig); hold(ax,'on');

FS_axes   = 20;
FS_label  = 20;
FS_title  = 24;
FS_legend = 16;
FS_text   = 15;
LW        = 2.0;
MS        = 5;

% Main curves
h_apid = plot(ax, t_apid_hr, y_apid_meas, '-o', ...
    'LineWidth', LW, 'MarkerSize', MS, ...
    'DisplayName', 'APID measured');

h_rapid = plot(ax, t_rapid_hr, y_rapid_meas, '-o', ...
    'LineWidth', LW, 'MarkerSize', MS, ...
    'DisplayName', 'RAPID used measurement');

% Setpoint
h_set = yline(ax, T4_des, '--k', ...
    'LineWidth', 1.8, 'DisplayName', 'Setpoint');

% ±30% band
band_plot = band_pct_plot * T4_des;
band_lo_plot = T4_des - band_plot;
band_hi_plot = T4_des + band_plot;

bandColor = [0.2 0.8 1.0];
h_band = yline(ax, band_lo_plot, ':', ...
    'LineWidth', 2.0, 'Color', bandColor, ...
    'DisplayName', sprintf('\\pm %.0f%% band', 100*band_pct_plot));
yline(ax, band_hi_plot, ':', ...
    'LineWidth', 2.0, 'Color', bandColor, ...
    'HandleVisibility', 'off');

% ±10% band, light gray, no legend
band10 = band_pct_control * T4_des;
yline(ax, T4_des - band10, ':', ...
    'Color', [0.7 0.7 0.7], 'LineWidth', 1.4, 'HandleVisibility', 'off');
yline(ax, T4_des + band10, ':', ...
    'Color', [0.7 0.7 0.7], 'LineWidth', 1.4, 'HandleVisibility', 'off');

% Axes cosmetics
tmax_hr = max([t_apid_hr(:); t_rapid_hr(:)]);
xlim(ax, [0 tmax_hr]);
ylim(ax, [0 35]);

grid(ax,'on');
set(ax, 'FontSize', FS_axes);
xlabel(ax, 'Time (h)', 'FontSize', FS_label);
ylabel(ax, 'Measured T_4^{ext} (a.u.)', 'FontSize', FS_label);
title(ax, sprintf('Measured T4 trajectories: APID vs RAPID (setpoint = %g)', T4_des), ...
    'FontSize', FS_title);

lgd = legend(ax, [h_apid h_rapid h_set h_band], 'Location', 'southeast');
lgd.FontSize = FS_legend;
lgd.Color = 'white';
lgd.Box = 'on';
lgd.EdgeColor = [0.7 0.7 0.7];

% Text block
txt = sprintf([ ...
    'APID : OS=%.1f%%, Rise=%.2f h, Settle=%.2f h, InBand(%.0f%%)=%.1f%%, InBand10%%=%.1f%%, IAE=%.2f\n' ...
    'RAPID: OS=%.1f%%, Rise=%.2f h, Settle=%.2f h, InBand(%.0f%%)=%.1f%%, InBand10%%=%.1f%%, IAE=%.2f'], ...
    M_apid.Overshoot_pct, M_apid.RiseTime_hr, M_apid.SettleTime_hr, ...
    100*band_pct_metrics, M_apid.TimeInBand_pct, M_apid.TimeInBand10_pct, M_apid.IAE, ...
    M_rapid.Overshoot_pct, M_rapid.RiseTime_hr, M_rapid.SettleTime_hr, ...
    100*band_pct_metrics, M_rapid.TimeInBand_pct, M_rapid.TimeInBand10_pct, M_rapid.IAE);

text(ax, 0.01, 0.02, txt, ...
    'Units', 'normalized', ...
    'VerticalAlignment', 'bottom', ...
    'HorizontalAlignment', 'left', ...
    'FontName', 'monospace', ...
    'FontSize', FS_text, ...
    'BackgroundColor', 'white', ...
    'EdgeColor', 'none', ...
    'Margin', 4);

% ------------------------------------------------------------
% Printed summary
% ------------------------------------------------------------
fprintf('\n=== Measured T4 comparison summary ===\n');
fprintf('APID  final measured T4   = %.3f\n', y_apid_meas(end));
fprintf('RAPID final measured T4   = %.3f\n', y_rapid_meas(end));
fprintf('APID  max measured T4     = %.3f\n', max(y_apid_meas));
fprintf('RAPID max measured T4     = %.3f\n', max(y_rapid_meas));
fprintf('APID  overshoot           = %.2f %%\n', M_apid.Overshoot_pct);
fprintf('RAPID overshoot           = %.2f %%\n', M_rapid.Overshoot_pct);
fprintf('APID  rise time           = %.2f h\n', M_apid.RiseTime_hr);
fprintf('RAPID rise time           = %.2f h\n', M_rapid.RiseTime_hr);
fprintf('APID  settling time       = %.2f h\n', M_apid.SettleTime_hr);
fprintf('RAPID settling time       = %.2f h\n', M_rapid.SettleTime_hr);
fprintf('APID  time in ±30%% band   = %.2f %%\n', M_apid.TimeInBand_pct);
fprintf('RAPID time in ±30%% band   = %.2f %%\n', M_rapid.TimeInBand_pct);
fprintf('APID  time in ±10%% band   = %.2f %%\n', M_apid.TimeInBand10_pct);
fprintf('RAPID time in ±10%% band   = %.2f %%\n', M_rapid.TimeInBand10_pct);
fprintf('APID  IAE (measured)      = %.3f\n', M_apid.IAE);
fprintf('RAPID IAE (used meas.)    = %.3f\n', M_rapid.IAE);

% ============================================================
% Local helper
% ============================================================
function M = compute_measured_metrics(t_min, y, r, band_pct_metrics, band_pct_control)

t_min = t_min(:);
y     = y(:);
t_hr  = t_min / 60;

M.Overshoot_pct = 100 * max(0, (max(y) - r) / max(r, eps));

idx_rise = find(y >= 0.9*r, 1, 'first');
if isempty(idx_rise)
    M.RiseTime_hr = NaN;
else
    M.RiseTime_hr = t_hr(idx_rise);
end

bandM = band_pct_metrics * r;
inbandM = abs(y - r) <= bandM;
M.TimeInBand_pct = 100 * mean(inbandM);

bandC = band_pct_control * r;
inbandC = abs(y - r) <= bandC;
M.TimeInBand10_pct = 100 * mean(inbandC);

M.SettleTime_hr = NaN;
for i = 1:numel(inbandM)
    if all(inbandM(i:end))
        M.SettleTime_hr = t_hr(i);
        break;
    end
end

M.IAE = trapz(t_hr, abs(y - r));
end