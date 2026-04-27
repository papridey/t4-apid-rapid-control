clear; clc; close all;

%% =========================================================
% User settings
% ==========================================================
T4_des = 25;

% Choose adaptive controller to compare against:
% controller_mode = 'APID';
% controller_mode = 'RAPID';
controller_mode = 'APID';

fixed_opts = struct( ...
    'Kp', 0.90, ...
    'Ki', 0.010, ...
    'Kd', 0.00, ...
    't_end_hr', 80, ...
    'meas_sigma', 0.0);

%% =========================================================
% Run controllers
% ==========================================================
fprintf('\nRunning fixed PID baseline...\n');
OUT_fix = run_pid_fixed_baseline(T4_des, fixed_opts);

switch upper(controller_mode)
    case 'APID'
        fprintf('Running APID...\n');
        OUT_cmp = run_apid_bandlock_smooth(T4_des);
        cmp_name = 'APID';
    case 'RAPID'
        fprintf('Running RAPID...\n');
        OUT_cmp = run_rapid_clean5(T4_des);
        cmp_name = 'RAPID';
    otherwise
        error('Unknown controller_mode. Use ''APID'' or ''RAPID''.');
end

%% =========================================================
% Metrics
% ==========================================================
M_fix = compute_metrics_from_out(OUT_fix);
M_cmp = compute_metrics_from_out(OUT_cmp);

T = table( ...
    ["Fixed PID"; string(cmp_name)], ...
    [M_fix.overshoot_pct;    M_cmp.overshoot_pct], ...
    [M_fix.rise_time_hr;     M_cmp.rise_time_hr], ...
    [M_fix.settling_time_hr; M_cmp.settling_time_hr], ...
    [M_fix.time_in_30pct;    M_cmp.time_in_30pct], ...
    [M_fix.time_in_10pct;    M_cmp.time_in_10pct], ...
    [M_fix.IAE;              M_cmp.IAE], ...
    [M_fix.control_energy;   M_cmp.control_energy], ...
    [M_fix.ef_sat_pct;       M_cmp.ef_sat_pct], ...
    'VariableNames', {'Controller','Overshoot_pct','RiseTime_h','SettlingTime_h', ...
                      'TimeIn30pct_pct','TimeIn10pct_pct','IAE','ControlEnergy','EFSat_pct'});
disp(T);

%% =========================================================
% Main comparison figure
% ==========================================================
figure('Color','w','Position',[100 100 1450 950]);

% ---------------------------------------------------------
% Measured output
% ---------------------------------------------------------
subplot(2,2,1); hold on;
plot_measured_series(OUT_fix, 'Fixed PID');
plot_measured_series(OUT_cmp, cmp_name);
yline(T4_des, '--k', 'LineWidth', 1.4);
yline(1.30*T4_des, ':k', 'LineWidth', 1.0);
yline(0.70*T4_des, ':k', 'LineWidth', 1.0);
xlabel('Time (h)');
ylabel('Measured T_4^{ext} (a.u.)');
title('Measured tracking comparison');
legend('Location','southeast');
grid on; box on;

% ---------------------------------------------------------
% True output
% ---------------------------------------------------------
subplot(2,2,2); hold on;
plot(get_time_hr(OUT_fix), get_true_output(OUT_fix), 'LineWidth',1.6);
plot(get_time_hr(OUT_cmp), get_true_output(OUT_cmp), 'LineWidth',1.6);
yline(T4_des, '--k', 'LineWidth',1.4);
xlabel('Time (h)');
ylabel('True T_4^{ext} (a.u.)');
title('True output comparison');
legend('Fixed PID', cmp_name, 'Setpoint', 'Location','southeast');
grid on; box on;

% ---------------------------------------------------------
% EF command
% ---------------------------------------------------------
subplot(2,2,3); hold on;
[t_fix_ef, u_fix] = get_ef_series(OUT_fix);
[t_cmp_ef, u_cmp] = get_ef_series(OUT_cmp);
stairs(t_fix_ef, u_fix, 'LineWidth',1.6);
stairs(t_cmp_ef, u_cmp, 'LineWidth',1.6);
xlabel('Time (h)');
ylabel('EF command (a.u.)');
title('EF command comparison');
legend('Fixed PID', cmp_name, 'Location','southeast');
grid on; box on;

% ---------------------------------------------------------
% Amplitude
% ---------------------------------------------------------
subplot(2,2,4); hold on;
[t_fix_A, A_fix] = get_amp_series(OUT_fix);
[t_cmp_A, A_cmp] = get_amp_series(OUT_cmp);
stairs(t_fix_A, A_fix, 'LineWidth',1.6);
stairs(t_cmp_A, A_cmp, 'LineWidth',1.6);
xlabel('Time (h)');
ylabel('Applied amplitude (a.u.)');
title('Per-window / held amplitude comparison');
legend('Fixed PID', cmp_name, 'Location','southeast');
grid on; box on;

sgtitle(sprintf('Fixed PID baseline vs %s for T_4^{des} = %.1f', cmp_name, T4_des));

%% =========================================================
% Optional metric bars
% ==========================================================
figure('Color','w','Position',[120 120 1200 650]);

controllers = categorical({'Fixed PID', cmp_name});
controllers = reordercats(controllers, {'Fixed PID', cmp_name});

subplot(2,3,1);
bar(controllers, [M_fix.rise_time_hr, M_cmp.rise_time_hr]);
ylabel('Hours'); title('Rise time'); grid on;

subplot(2,3,2);
bar(controllers, [M_fix.settling_time_hr, M_cmp.settling_time_hr]);
ylabel('Hours'); title('Settling time'); grid on;

subplot(2,3,3);
bar(controllers, [M_fix.overshoot_pct, M_cmp.overshoot_pct]);
ylabel('%'); title('Overshoot'); grid on;

subplot(2,3,4);
bar(controllers, [M_fix.IAE, M_cmp.IAE]);
ylabel('IAE'); title('Integral absolute error'); grid on;

subplot(2,3,5);
bar(controllers, [M_fix.control_energy, M_cmp.control_energy]);
ylabel('E_u'); title('Control energy'); grid on;

subplot(2,3,6);
bar(controllers, [M_fix.time_in_10pct, M_cmp.time_in_10pct]);
ylabel('%'); title('Time in 10% band'); grid on;

sgtitle(sprintf('Metric summary: Fixed PID vs %s', cmp_name));

%% =========================================================
% Local functions
% ==========================================================
function M = compute_metrics_from_out(OUT)
    t_meas_hr = get_t_meas_hr(OUT);
    z_meas    = OUT.T4_meas(:);
    ref       = extract_setpoint(OUT);

    err = ref - z_meas;

    zmax = max(z_meas);
    M.overshoot_pct = 100 * max(0, zmax - ref) / ref;

    i10 = find(z_meas >= 0.10*ref, 1, 'first');
    i90 = find(z_meas >= 0.90*ref, 1, 'first');
    if ~isempty(i10) && ~isempty(i90) && i90 >= i10
        M.rise_time_hr = t_meas_hr(i90) - t_meas_hr(i10);
    else
        M.rise_time_hr = NaN;
    end

    idx_settle = NaN;
    for k = 1:numel(z_meas)
        if all(abs(z_meas(k:end) - ref) <= 0.05*ref)
            idx_settle = k;
            break;
        end
    end
    if isnan(idx_settle)
        M.settling_time_hr = NaN;
    else
        M.settling_time_hr = t_meas_hr(idx_settle);
    end

    M.time_in_30pct = 100 * mean(abs(z_meas - ref) <= 0.30*ref);
    M.time_in_10pct = 100 * mean(abs(z_meas - ref) <= 0.10*ref);
    M.IAE = trapz(t_meas_hr, abs(err));

    [~, u] = get_ef_series(OUT);
    Ts_min = extract_Ts(OUT);
    M.control_energy = sum((u(:).^2) * (Ts_min/60));

    EF_max = extract_EFmax(OUT);
    if isnan(EF_max)
        M.ef_sat_pct = NaN;
    else
        M.ef_sat_pct = 100 * mean(abs(u(:) - EF_max) < 1e-10);
    end
end

function ref = extract_setpoint(OUT)
    if isfield(OUT, 'setpoint')
        ref = OUT.setpoint;
    elseif isfield(OUT, 'T4_des')
        ref = OUT.T4_des;
    elseif isfield(OUT, 'params') && isstruct(OUT.params) && isfield(OUT.params, 'T4_des')
        ref = OUT.params.T4_des;
    elseif isfield(OUT, 'P') && isstruct(OUT.P) && isfield(OUT.P, 'T4_des')
        ref = OUT.P.T4_des;
    elseif isfield(OUT, 'P') && isstruct(OUT.P) && isfield(OUT.P, 'setpoint')
        ref = OUT.P.setpoint;
    else
        error(['Could not find setpoint in OUT. Expected one of: ', ...
               'OUT.setpoint, OUT.T4_des, OUT.params.T4_des, OUT.P.T4_des.']);
    end
end

function t_meas_hr = get_t_meas_hr(OUT)
    t = OUT.t_meas(:);

    % Fixed baseline stores measurement time in hours.
    % APID stores t_meas in minutes.
    if isfield(OUT,'time_hr') && isfield(OUT,'time')
        % If max(t_meas) is comparable to max(time), assume minutes.
        if max(t) > 1.5 * max(OUT.time_hr(:))
            t_meas_hr = t / 60;
        else
            t_meas_hr = t;
        end
    else
        % Fallback heuristic
        if max(t) > 200
            t_meas_hr = t / 60;
        else
            t_meas_hr = t;
        end
    end
end

function plot_measured_series(OUT, labeltxt)
    t_meas_hr = get_t_meas_hr(OUT);
    plot(t_meas_hr, OUT.T4_meas(:), '-o', ...
        'LineWidth',1.4, 'MarkerSize',3.5, 'DisplayName', labeltxt);
end

function t = get_time_hr(OUT)
    if isfield(OUT,'time_hr')
        t = OUT.time_hr(:);
    elseif isfield(OUT,'time')
        t = OUT.time(:) / 60;
    else
        error('Could not find time vector in OUT.');
    end
end

function y = get_true_output(OUT)
    if isfield(OUT,'T4_true')
        y = OUT.T4_true(:);
    elseif isfield(OUT,'T4_log')
        y = OUT.T4_log(:);
    elseif isfield(OUT,'y')
        if size(OUT.y,2) >= 4
            y = OUT.y(:,4);
        else
            error('OUT.y exists but does not contain 4 columns.');
        end
    else
        error('Could not find true output in OUT.');
    end
end

function [t_hr, u] = get_ef_series(OUT)
    % APID logs u_pid on the fine simulation grid.
    if isfield(OUT,'u_pid')
        u = OUT.u_pid(:);
        t_hr = get_time_hr(OUT);
    % Some files may use uPID or u_cmd on fine grid
    elseif isfield(OUT,'uPID') && numel(OUT.uPID) > 10
        u = OUT.uPID(:);
        t_hr = get_time_hr(OUT);
    % Fixed baseline logs EF_cmd windowwise
    elseif isfield(OUT,'EF_cmd')
        u = OUT.EF_cmd(:);
        Ts_min = extract_Ts(OUT);
        t_hr = (0:numel(u)-1)' * Ts_min / 60;
    elseif isfield(OUT,'u_cmd')
        u = OUT.u_cmd(:);
        Ts_min = extract_Ts(OUT);
        t_hr = (0:numel(u)-1)' * Ts_min / 60;
    else
        error('Could not find EF command field.');
    end
end

function [t_hr, A] = get_amp_series(OUT)
    % APID A_t is on fine grid; A_hist is windowwise.
    if isfield(OUT,'A_t')
        A = OUT.A_t(:);
        t_hr = get_time_hr(OUT);
    elseif isfield(OUT,'A_cmd')
        A = OUT.A_cmd(:);
        Ts_min = extract_Ts(OUT);
        t_hr = (0:numel(A)-1)' * Ts_min / 60;
    elseif isfield(OUT,'A_hist')
        A = OUT.A_hist(:);
        mask = ~isnan(A);
        A = A(mask);
        if isfield(OUT,'sample_times')
            t_hr = OUT.sample_times(mask)' / 60;
        else
            Ts_min = extract_Ts(OUT);
            t_hr = (0:numel(A)-1)' * Ts_min / 60;
        end
    elseif isfield(OUT,'A_applied')
        A = OUT.A_applied(:);
        Ts_min = extract_Ts(OUT);
        t_hr = (0:numel(A)-1)' * Ts_min / 60;
    elseif isfield(OUT,'A_log')
        A = OUT.A_log(:);
        Ts_min = extract_Ts(OUT);
        t_hr = (0:numel(A)-1)' * Ts_min / 60;
    elseif isfield(OUT,'A_fixed')
        A = OUT.A_fixed(:);
        Ts_min = extract_Ts(OUT);
        t_hr = (0:numel(A)-1)' * Ts_min / 60;
    else
        error('Could not find amplitude field.');
    end
end

function Ts_min = extract_Ts(OUT)
    if isfield(OUT,'P') && isfield(OUT.P,'Ts_min')
        Ts_min = OUT.P.Ts_min;
    elseif isfield(OUT,'params') && isfield(OUT.params,'Ts_min')
        Ts_min = OUT.params.Ts_min;
    elseif isfield(OUT,'Ts_min')
        Ts_min = OUT.Ts_min;
    elseif isfield(OUT,'Ts')
        Ts_min = OUT.Ts;
    elseif isfield(OUT,'sample_times') && numel(OUT.sample_times) >= 2
        Ts_min = median(diff(OUT.sample_times));
    else
        Ts_min = 120;
    end
end

function EF_max = extract_EFmax(OUT)
    if isfield(OUT,'P') && isfield(OUT.P,'EF_max')
        EF_max = OUT.P.EF_max;
    elseif isfield(OUT,'params') && isfield(OUT.params,'EF_max')
        EF_max = OUT.params.EF_max;
    elseif isfield(OUT,'EF_max')
        EF_max = OUT.EF_max;
    else
        EF_max = NaN;
    end
end