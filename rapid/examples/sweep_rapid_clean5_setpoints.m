clear; clc; close all;

% ============================================================
% Sweep settings
% ============================================================
setpoints = [15 20, 25, 30, 35, 45];

base_opts = struct( ...
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

% ============================================================
% Storage
% ============================================================
Nsp = numel(setpoints);
OUTS = cell(Nsp,1);

summary = struct();
summary.setpoint          = setpoints(:);
summary.final_used        = nan(Nsp,1);
summary.max_used          = nan(Nsp,1);
summary.final_true        = nan(Nsp,1);
summary.max_true          = nan(Nsp,1);
summary.time_in_band_5    = nan(Nsp,1);
summary.time_in_band_10   = nan(Nsp,1);
summary.IAE_used          = nan(Nsp,1);
summary.final_A           = nan(Nsp,1);
summary.max_A             = nan(Nsp,1);
summary.final_cost        = nan(Nsp,1);
summary.max_uPID          = nan(Nsp,1);

fprintf('\n=== Running RAPID sweep over setpoints ===\n');

% ============================================================
% Main sweep
% ============================================================
for j = 1:Nsp
    T4_des = setpoints(j);

    fprintf('Running setpoint %g ...\n', T4_des);
    OUT = run_rapid_clean5(T4_des, base_opts);
    OUTS{j} = OUT;

    % Measured/used signal
    mask_meas = ~isnan(OUT.t_meas) & ~isnan(OUT.T4_meas_used);
    t_meas_hr = OUT.t_meas(mask_meas) / 60;
    y_used    = OUT.T4_meas_used(mask_meas);

    % True signal sampled at measurement instants for comparable metrics
    y_true_samp = interp1(OUT.time, OUT.T4_log, OUT.t_meas(mask_meas), 'linear');

    % Amplitude / cost masks
    maskA = ~isnan(OUT.A_hist);
    maskJ = ~isnan(OUT.cost_hist);

    % Metrics
    summary.final_used(j)      = y_used(end);
    summary.max_used(j)        = max(y_used);
    summary.final_true(j)      = OUT.T4_log(end);
    summary.max_true(j)        = max(OUT.T4_log);
    summary.time_in_band_5(j)  = mean(abs(y_used - T4_des) <= 0.05*T4_des);
    summary.time_in_band_10(j) = mean(abs(y_used - T4_des) <= 0.10*T4_des);
    summary.IAE_used(j)        = trapz(t_meas_hr, abs(y_used - T4_des));

    if any(maskA)
        Avals = OUT.A_hist(maskA);
        summary.final_A(j) = Avals(end);
        summary.max_A(j)   = max(Avals);
    end

    if any(maskJ)
        Jvals = OUT.cost_hist(maskJ);
        summary.final_cost(j) = Jvals(end);
    end

    summary.max_uPID(j) = max(OUT.u_pid);

    fprintf(['  final used = %.3f | max used = %.3f | final true = %.3f | ' ...
             'band5 = %.2f%% | band10 = %.2f%% | IAE = %.3f\n'], ...
             summary.final_used(j), summary.max_used(j), summary.final_true(j), ...
             100*summary.time_in_band_5(j), 100*summary.time_in_band_10(j), ...
             summary.IAE_used(j));
end

fprintf('Done.\n');

% ============================================================
% Display summary table
% ============================================================
T = table(summary.setpoint, summary.final_used, summary.max_used, ...
          summary.final_true, summary.max_true, ...
          100*summary.time_in_band_5, 100*summary.time_in_band_10, ...
          summary.IAE_used, summary.final_A, summary.max_A, ...
          summary.final_cost, summary.max_uPID, ...
    'VariableNames', {'Setpoint','FinalUsed','MaxUsed','FinalTrue','MaxTrue', ...
                      'Band5Pct','Band10Pct','IAEUsed','FinalA','MaxA', ...
                      'FinalCost','MaxuPID'});

disp(T);

% ============================================================
% Overlay plot: used measurements
% ============================================================
figure('Color','w','Position',[100 100 1400 950]);

subplot(2,2,1); hold on;
for j = 1:Nsp
    OUT = OUTS{j};
    mask_meas = ~isnan(OUT.t_meas) & ~isnan(OUT.T4_meas_used);
    stairs(OUT.t_meas(mask_meas)/60, OUT.T4_meas_used(mask_meas), 'LineWidth', 1.5, ...
        'DisplayName', sprintf('SP=%g', setpoints(j)));
end
xlabel('Time (h)');
ylabel('Used measurement');
title('Used measurement vs time');
grid on;
legend('Location','best');

% ============================================================
% Overlay plot: true outputs
% ============================================================
subplot(2,2,2); hold on;
for j = 1:Nsp
    OUT = OUTS{j};
    plot(OUT.time/60, OUT.T4_log, 'LineWidth', 1.5, ...
        'DisplayName', sprintf('SP=%g', setpoints(j)));
end
xlabel('Time (h)');
ylabel('True T_4^{ext}');
title('True output vs time');
grid on;
legend('Location','best');

% ============================================================
% Overlay plot: amplitudes
% ============================================================
subplot(2,2,3); hold on;
for j = 1:Nsp
    OUT = OUTS{j};
    maskA = ~isnan(OUT.A_hist);
    stairs(OUT.sample_times(maskA)/60, OUT.A_hist(maskA), 'LineWidth', 1.5, ...
        'DisplayName', sprintf('SP=%g', setpoints(j)));
end
xlabel('Time (h)');
ylabel('Amplitude A');
title('Commanded amplitude');
grid on;
legend('Location','best');

% ============================================================
% Overlay plot: robust costs
% ============================================================
subplot(2,2,4); hold on;
for j = 1:Nsp
    OUT = OUTS{j};
    maskJ = ~isnan(OUT.cost_hist);
    stairs(OUT.sample_times(maskJ)/60, OUT.cost_hist(maskJ), 'LineWidth', 1.5, ...
        'DisplayName', sprintf('SP=%g', setpoints(j)));
end
xlabel('Time (h)');
ylabel('J_{rob}');
title('Robust cost');
grid on;
legend('Location','best');

sgtitle('RAPID sweep over setpoints');

% ============================================================
% Metric plots
% ============================================================
figure('Color','w','Position',[120 120 1400 850]);

subplot(2,3,1);
plot(summary.setpoint, summary.final_used, '-o', 'LineWidth', 1.6, 'MarkerSize', 7);
xlabel('Setpoint');
ylabel('Final used measurement');
title('Final used output');
grid on;

subplot(2,3,2);
plot(summary.setpoint, 100*summary.time_in_band_5, '-o', 'LineWidth', 1.6, 'MarkerSize', 7);
xlabel('Setpoint');
ylabel('Time in \pm5% band (%)');
title('Band performance');
grid on;

subplot(2,3,3);
plot(summary.setpoint, summary.IAE_used, '-o', 'LineWidth', 1.6, 'MarkerSize', 7);
xlabel('Setpoint');
ylabel('IAE');
title('Integrated absolute error');
grid on;

subplot(2,3,4);
plot(summary.setpoint, summary.max_A, '-o', 'LineWidth', 1.6, 'MarkerSize', 7);
xlabel('Setpoint');
ylabel('Max amplitude');
title('Maximum A');
grid on;

subplot(2,3,5);
plot(summary.setpoint, summary.max_uPID, '-o', 'LineWidth', 1.6, 'MarkerSize', 7);
xlabel('Setpoint');
ylabel('Max u_{PID}');
title('Maximum command');
grid on;

subplot(2,3,6);
plot(summary.setpoint, summary.final_cost, '-o', 'LineWidth', 1.6, 'MarkerSize', 7);
xlabel('Setpoint');
ylabel('Final robust cost');
title('Final J_{rob}');
grid on;

sgtitle('RAPID sweep summary metrics');

% ============================================================
% Optional save
% ============================================================
save_results = false;
if save_results
    save('rapid_clean5_sweep_results.mat', 'OUTS', 'summary', 'T', 'setpoints', 'base_opts');
end