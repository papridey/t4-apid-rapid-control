clear; clc; close all;

% ------------------------------------------------------------
% User settings
% ------------------------------------------------------------
T4_des = 25;

opts_apid = struct( ...
    'dt_seconds', 0.1, ...
    'sim_hours', 80, ...
    'make_plots', false, ...
    'verbose', false, ...
    'do_save_pdf', false);

opts_rapid = struct( ...
    'dt_seconds', 0.1, ...
    'sim_hours', 80, ...
    'do_plots', false, ...
    'verbose', false, ...
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

% ------------------------------------------------------------
% Run controllers
% ------------------------------------------------------------
%OUT_apid  = run_apid_bandlock_smooth(T4_des, opts_apid);
%OUT_rapid = run_rapid_clean5(T4_des, opts_rapid);
fprintf('Starting APID...\n'); drawnow;
OUT_apid = run_apid_bandlock_smooth(T4_des, opts_apid);

fprintf('Finished APID. Starting RAPID...\n'); drawnow;
OUT_rapid = run_rapid_clean5(T4_des, opts_rapid);

fprintf('Finished RAPID. Building comparison plots...\n'); drawnow;

% ------------------------------------------------------------
% Extract comparable measured signals
% ------------------------------------------------------------
% APID: measured signal is the sampled T4_meas
mask_apid = ~isnan(OUT_apid.t_meas) & ~isnan(OUT_apid.T4_meas);
t_apid_hr = OUT_apid.t_meas(mask_apid) / 60;
y_apid_meas = OUT_apid.T4_meas(mask_apid);

% RAPID: use the corrected/used measurement
mask_rapid = ~isnan(OUT_rapid.t_meas) & ~isnan(OUT_rapid.T4_meas_used);
t_rapid_hr = OUT_rapid.t_meas(mask_rapid) / 60;
y_rapid_meas = OUT_rapid.T4_meas_used(mask_rapid);

% True/continuous outputs
t_apid_true_hr  = OUT_apid.time / 60;
y_apid_true     = OUT_apid.T4_log;

t_rapid_true_hr = OUT_rapid.time / 60;
y_rapid_true    = OUT_rapid.T4_log;

% Commands
u_apid_t_hr  = OUT_apid.time / 60;
u_apid       = OUT_apid.u_pid;

u_rapid_t_hr = OUT_rapid.time / 60;
u_rapid      = OUT_rapid.u_pid;

% Amplitudes
maskA_apid = ~isnan(OUT_apid.A_hist);
tA_apid_hr = OUT_apid.sample_times(maskA_apid) / 60;
A_apid     = OUT_apid.A_hist(maskA_apid);

maskA_rapid = ~isnan(OUT_rapid.A_hist);
tA_rapid_hr = OUT_rapid.sample_times(maskA_rapid) / 60;
A_rapid     = OUT_rapid.A_hist(maskA_rapid);

% Gains
tg_apid_hr  = OUT_apid.sample_times / 60;
tg_rapid_hr = OUT_rapid.sample_times / 60;

% Costs
maskJ_apid = ~isnan(OUT_apid.cost_hist);
tJ_apid_hr = OUT_apid.sample_times(maskJ_apid) / 60;
J_apid     = OUT_apid.cost_hist(maskJ_apid);

maskJ_rapid = ~isnan(OUT_rapid.cost_hist);
tJ_rapid_hr = OUT_rapid.sample_times(maskJ_rapid) / 60;
J_rapid     = OUT_rapid.cost_hist(maskJ_rapid);

% ------------------------------------------------------------
% Band for plotting
% ------------------------------------------------------------
band_pct = 0.05;
band_hi = T4_des * (1 + band_pct);
band_lo = T4_des * (1 - band_pct);

% ------------------------------------------------------------
% Overlay figure
% ------------------------------------------------------------
figure('Color','w','Position',[100 100 1600 1150]);

% 1) Tracking
subplot(3,2,1); hold on;
xx = linspace(0, max([t_apid_true_hr(end), t_rapid_true_hr(end)]), 500);
patch([xx fliplr(xx)], [band_lo*ones(size(xx)) fliplr(band_hi*ones(size(xx)))], ...
      [0.92 0.92 0.92], 'EdgeColor','none', 'FaceAlpha',0.5, ...
      'DisplayName','\pm5% band');
plot(xx, T4_des*ones(size(xx)), '--k', 'LineWidth',1.5, 'DisplayName','Setpoint');

plot(t_apid_true_hr,  y_apid_true,  '-',  'LineWidth',1.5, 'DisplayName','APID true');
plot(t_rapid_true_hr, y_rapid_true, '-',  'LineWidth',1.5, 'DisplayName','RAPID true');

stairs(t_apid_hr,  y_apid_meas,  '-', 'LineWidth',1.3, 'DisplayName','APID measured');
stairs(t_rapid_hr, y_rapid_meas, '-', 'LineWidth',1.3, 'DisplayName','RAPID used measurement');

xlabel('Time (h)');
ylabel('T_4^{ext}');
title('Tracking comparison');
grid on;
legend('Location','best');

% 2) PID command
subplot(3,2,2); hold on;
plot(u_apid_t_hr,  u_apid,  '-', 'LineWidth',1.4, 'DisplayName','APID');
plot(u_rapid_t_hr, u_rapid, '-', 'LineWidth',1.4, 'DisplayName','RAPID');
xlabel('Time (h)');
ylabel('u_{PID}');
title('PID command');
grid on;
legend('Location','best');

% 3) EF supplied
subplot(3,2,3); hold on;
plot(OUT_apid.time/60,  OUT_apid.EF_supplied_log,  '-', 'LineWidth',1.4, 'DisplayName','APID');
plot(OUT_rapid.time/60, OUT_rapid.EF_supplied_log, '-', 'LineWidth',1.4, 'DisplayName','RAPID');
xlabel('Time (h)');
ylabel('EF supplied');
title('EF supplied');
grid on;
legend('Location','best');

% 4) Amplitude
subplot(3,2,4); hold on;
stairs(tA_apid_hr,  A_apid,  '-', 'LineWidth',1.4, 'DisplayName','APID');
stairs(tA_rapid_hr, A_rapid, '-', 'LineWidth',1.4, 'DisplayName','RAPID');
xlabel('Time (h)');
ylabel('Amplitude A');
title('Per-window amplitude');
grid on;
legend('Location','best');

% 5) Kp comparison
subplot(3,2,5); hold on;
stairs(tg_apid_hr,  OUT_apid.Kp_hist,  '-', 'LineWidth',1.4, 'DisplayName','APID K_p');
stairs(tg_rapid_hr, OUT_rapid.Kp_hist, '-', 'LineWidth',1.4, 'DisplayName','RAPID K_p');
stairs(tg_apid_hr,  OUT_apid.Ki_hist,  '--', 'LineWidth',1.2, 'DisplayName','APID K_i');
stairs(tg_rapid_hr, OUT_rapid.Ki_hist, '--', 'LineWidth',1.2, 'DisplayName','RAPID K_i');
stairs(tg_apid_hr,  OUT_apid.Kd_hist,  ':', 'LineWidth',1.2, 'DisplayName','APID K_d');
stairs(tg_rapid_hr, OUT_rapid.Kd_hist, ':', 'LineWidth',1.2, 'DisplayName','RAPID K_d');
xlabel('Time (h)');
ylabel('Gains');
title('Adaptive gains');
grid on;
legend('Location','best');

% 6) Cost
subplot(3,2,6); hold on;
stairs(tJ_apid_hr,  J_apid,  '-', 'LineWidth',1.4, 'DisplayName','APID');
stairs(tJ_rapid_hr, J_rapid, '-', 'LineWidth',1.4, 'DisplayName','RAPID');
xlabel('Time (h)');
ylabel('Cost');
title('Per-window cost');
grid on;
legend('Location','best');

sgtitle(sprintf('APID vs RAPID comparison (setpoint = %g)', T4_des));

% ------------------------------------------------------------
% Optional summary metrics
% ------------------------------------------------------------
band_apid  = abs(y_apid_meas  - T4_des) <= band_pct*T4_des;
band_rapid = abs(y_rapid_meas - T4_des) <= band_pct*T4_des;

IAE_apid  = trapz(t_apid_hr,  abs(y_apid_meas  - T4_des));
IAE_rapid = trapz(t_rapid_hr, abs(y_rapid_meas - T4_des));

fprintf('\n=== Comparison summary ===\n');
fprintf('APID  final measured T4   = %.3f\n', y_apid_meas(end));
fprintf('RAPID final measured T4   = %.3f\n', y_rapid_meas(end));
fprintf('APID  max measured T4     = %.3f\n', max(y_apid_meas));
fprintf('RAPID max measured T4     = %.3f\n', max(y_rapid_meas));
fprintf('APID  time in ±5%% band    = %.2f %%\n', 100*mean(band_apid));
fprintf('RAPID time in ±5%% band    = %.2f %%\n', 100*mean(band_rapid));
fprintf('APID  IAE (measured)      = %.3f\n', IAE_apid);
fprintf('RAPID IAE (used measured) = %.3f\n', IAE_rapid);