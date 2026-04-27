function OUT = run_pid_fixed_baseline(T4_des, opts)
%RUN_PID_BASELINE
% Fixed-gain sampled-data PID baseline using the unchanged
% ode_constant_coeff_model.m plant.
%
% This baseline keeps:
%   - the same 16-state plant
%   - the same burst-averaged actuator structure
%   - the same sampled-data measurement/update cadence
%   - EF saturation, rate limits, and anti-windup
%
% It removes:
%   - gain adaptation
%   - band-lock / basal holding
%   - robust scenario-CVaR update
%   - bias correction
%
% Usage:
%   OUT = run_pid_baseline(25);
%   OUT = run_pid_baseline(25, struct('Kp',0.9,'Ki',0.01,'Kd',0.0));

if nargin < 2
    opts = struct();
end

rng(1); % reproducible measurement noise if used

% ------------------------------------------------------------
% Defaults / parameters
% ------------------------------------------------------------
P = get_defaults(opts);
P.T4_des = T4_des;

t_end_min = P.t_end_hr * 60;
tspan = 0:P.dt_min:t_end_min;
nSteps = numel(tspan);

% ------------------------------------------------------------
% Initial state
% ------------------------------------------------------------
y = zeros(numel(P.y0), nSteps);
y(:,1) = P.y0(:);

% ------------------------------------------------------------
% Controller states
% ------------------------------------------------------------
next_meas_time = P.T_measure_min;
next_ctrl_time = P.Ts_min;

I_state = 0.0;
d_f = 0.0;

z_true0 = y(P.idx_T4_ext,1);
z_used  = z_true0;
z_prev  = z_used;

EF_cmd = P.EF0;
A_cmd  = ef_to_amp(EF_cmd, P);

% ------------------------------------------------------------
% Logs
% ------------------------------------------------------------
t_meas   = 0;
T4_meas  = z_used;

e_log      = P.T4_des - z_used;
I_log      = I_state;
d_log      = d_f;
EF_cmd_log = EF_cmd;
A_cmd_log  = A_cmd;
K_hist     = [P.Kp, P.Ki, P.Kd];

% ------------------------------------------------------------
% Main loop
% ------------------------------------------------------------
for k = 1:(nSteps-1)
    tk  = tspan(k);
    tk1 = tspan(k+1);

    % Effective burst-averaged EF seen by plant during this fine step
    EF_now = burst_avg_EF(tk, A_cmd, P);

    % Integrate one fine step
    y(:,k+1) = rk4_step(@(tt,yy) plant_rhs(tt, yy, EF_now, P), tk, y(:,k), P.dt_min);

    % Optional nonnegativity safeguard
    y(:,k+1) = max(y(:,k+1), 0);

    % --------------------------------------------------------
    % Measurement update every T_measure_min
    % --------------------------------------------------------
    if tk1 >= next_meas_time - 1e-12
        z_true = y(P.idx_T4_ext, k+1);

        % noise only; no bias correction in this simple baseline
        z_used = z_true + P.meas_sigma * randn();
        z_used = max(z_used, 0);

        t_meas(end+1,1)  = tk1/60; %#ok<AGROW>
        T4_meas(end+1,1) = z_used; %#ok<AGROW>

        next_meas_time = next_meas_time + P.T_measure_min;
    end

    % --------------------------------------------------------
    % Controller update every Ts_min
    % --------------------------------------------------------
    if tk1 >= next_ctrl_time - 1e-12
        % sampled tracking error
        e_m = P.T4_des - z_used;

        % derivative-on-measurement
        d_m = (z_used - z_prev) / P.Ts_min;
        d_f = (1 - P.alpha_d) * d_f + P.alpha_d * d_m;

        % integral update
        I_state = (1 - P.I_leak) * I_state + e_m * P.Ts_min;
        I_state = clamp(I_state, -P.I_max, P.I_max);

        % discrete PID command
        D_m = -P.Kd * d_f;
        EF_unsat = P.Kp * e_m + P.Ki * I_state + D_m;

        % EF rate limit + saturation
        EF_rlim = rate_limit(EF_unsat, EF_cmd, P.dEF_max);
        EF_new  = clamp(EF_rlim, P.EF_min, P.EF_max);

        % anti-windup back-calculation
        I_state = I_state + P.Kb_aw * (EF_new - EF_unsat) * P.Ts_min;
        I_state = clamp(I_state, -P.I_max, P.I_max);

        % map EF to amplitude
        A_new = ef_to_amp(EF_new, P);
        A_new = rate_limit(A_new, A_cmd, P.dA_max);
        A_new = clamp(A_new, P.A_min, P.A_max);

        % commit
        EF_cmd = EF_new;
        A_cmd  = A_new;
        z_prev = z_used;

        % logs
        e_log(end+1,1)      = e_m; %#ok<AGROW>
        I_log(end+1,1)      = I_state; %#ok<AGROW>
        d_log(end+1,1)      = d_f; %#ok<AGROW>
        EF_cmd_log(end+1,1) = EF_cmd; %#ok<AGROW>
        A_cmd_log(end+1,1)  = A_cmd; %#ok<AGROW>
        K_hist(end+1,:)     = [P.Kp, P.Ki, P.Kd]; %#ok<AGROW>

        next_ctrl_time = next_ctrl_time + P.Ts_min;
    end
end

% ------------------------------------------------------------
% Output
% ------------------------------------------------------------
OUT.time_hr = tspan(:)/60;
OUT.y = y.';
OUT.T4_true = y(P.idx_T4_ext,:).';
OUT.t_meas = t_meas;
OUT.T4_meas = T4_meas;

OUT.e_meas = e_log;
OUT.I_log = I_log;
OUT.d_log = d_log;

OUT.EF_cmd = EF_cmd_log;
OUT.A_cmd  = A_cmd_log;
OUT.K_hist = K_hist;

OUT.setpoint = P.T4_des;
OUT.P = P;

end

% ========================================================================
% Helper functions
% ========================================================================

function P = get_defaults(opts)

% ------------------------
% Time settings
% ------------------------
P.t_end_hr      = get_opt(opts, 't_end_hr', 80);
P.dt_min        = get_opt(opts, 'dt_min', 0.1/60);   % 0.1 seconds in minutes
P.Ts_min        = get_opt(opts, 'Ts_min', 120);      % control update period
P.T_measure_min = get_opt(opts, 'T_measure_min', 20);

% ------------------------
% Initial condition / indexing
% ------------------------
P.idx_T4_ext = get_opt(opts, 'idx_T4_ext', 4);
P.y0 = get_opt(opts, 'y0', default_y0());

% ------------------------
% Plant parameters expected by unchanged ode_constant_coeff_model.m
% ------------------------
P.N                   = get_opt(opts, 'N', 10);
P.alpha1              = get_opt(opts, 'alpha1', 3.308);
P.K                   = get_opt(opts, 'K', 1.195);
P.n                   = get_opt(opts, 'n', 2);
P.gamma1              = get_opt(opts, 'gamma1', 0.0296);
P.a                   = get_opt(opts, 'a', 0.001);
P.alpha               = get_opt(opts, 'alpha', 168.7624);
P.gamma_NKX21         = get_opt(opts, 'gamma_NKX21', 0.01);
P.alpha_NKX21_offset  = get_opt(opts, 'alpha_NKX21_offset', 0.25);

P.kappa       = get_opt(opts, 'kappa', 1.0);
P.K2          = get_opt(opts, 'K2', 1000);
P.alpha_I     = get_opt(opts, 'alpha_I', 1.0);
P.gamma_I     = get_opt(opts, 'gamma_I', 0.004);
P.alpha_TG    = get_opt(opts, 'alpha_TG', 1.0);
P.gamma_Tg    = get_opt(opts, 'gamma_Tg', 0.04);
P.gammaint_T4 = get_opt(opts, 'gammaint_T4', 0.1504);
P.M_fun       = get_opt(opts, 'M_fun', 0.15);   % scalar, unchanged model expects scalar
P.gammaext_T4 = get_opt(opts, 'gammaext_T4', 0.01);

% ------------------------
% Fixed PID gains
% ------------------------
P.Kp = get_opt(opts, 'Kp', 0.90);
P.Ki = get_opt(opts, 'Ki', 0.010);
P.Kd = get_opt(opts, 'Kd', 0.00);

% ------------------------
% Measurement / filtering / anti-windup
% ------------------------
P.meas_sigma = get_opt(opts, 'meas_sigma', 0.0);
P.alpha_d    = get_opt(opts, 'alpha_d', 0.25);
P.I_leak     = get_opt(opts, 'I_leak', 0.00);
P.I_max      = get_opt(opts, 'I_max', 1e4);
P.Kb_aw      = get_opt(opts, 'Kb_aw', 1e-3);

% ------------------------
% EF / amplitude constraints
% ------------------------
P.EF_min  = get_opt(opts, 'EF_min', 0.0);
P.EF_max  = get_opt(opts, 'EF_max', 100.0);
P.EF0     = get_opt(opts, 'EF0', 0.0);

P.A_min   = get_opt(opts, 'A_min', 0.0);
P.A_max   = get_opt(opts, 'A_max', 0.10);

P.thr     = get_opt(opts, 'thr', 1e-4);
P.kA      = get_opt(opts, 'kA', 0.20);

P.dEF_max = get_opt(opts, 'dEF_max', 10.0);
P.dA_max  = get_opt(opts, 'dA_max', 0.01);

% ------------------------
% Burst timing parameters in minutes
% ------------------------
P.t1 = get_opt(opts, 't1', 500e-6/60);
P.t2 = get_opt(opts, 't2', 100e-6/60);
P.t3 = get_opt(opts, 't3', 500e-6/60);
P.t4 = get_opt(opts, 't4', 9.5e-4);
P.np = get_opt(opts, 'np', 500);
P.tgap   = get_opt(opts, 'tgap', 0.0);
P.t6_min = get_opt(opts, 't6_min', 20);

end

function y0 = default_y0()
y0 = zeros(16,1);
y0(1:4) = [4; 4; 4; 4];
end

function val = get_opt(S, name, default)
if isfield(S, name)
    val = S.(name);
else
    val = default;
end
end

function dydt = plant_rhs(t, y, EF_now, P)
% Keep ode_constant_coeff_model unchanged:
% it expects a function handle EF_time_min.
EF_time_min = @(tt) EF_now;
dydt = ode_constant_coeff_model(t, y, EF_time_min, P);
end

function y1 = rk4_step(f, t, y, h)
k1 = f(t, y);
k2 = f(t + 0.5*h, y + 0.5*h*k1);
k3 = f(t + 0.5*h, y + 0.5*h*k2);
k4 = f(t + h, y + h*k3);
y1 = y + (h/6) * (k1 + 2*k2 + 2*k3 + k4);
end

function out = clamp(x, lo, hi)
out = min(max(x, lo), hi);
end

function u_new = rate_limit(u_des, u_old, du_max)
du = u_des - u_old;
du = clamp(du, -du_max, du_max);
u_new = u_old + du;
end

function A = ef_to_amp(EF, P)
if EF < P.thr
    A = 0;
else
    A = clamp(P.kA * (EF - P.thr), P.A_min, P.A_max);
end
end

function EF = burst_avg_EF(t_min, A_cmd, P)
% Effective burst-averaged EF fed into the plant

Ts = P.Ts_min;
s  = mod(t_min, Ts);

Tp = P.t1 + P.t2 + P.t3 + P.t4;
delta_pulse = (P.t1 + P.t3) / Tp;

t5b = P.np * Tp;
Tb  = t5b + P.tgap;

nb = max(0, floor((Ts - P.t6_min) / Tb));

is_on = false;
for j = 0:(nb-1)
    if s >= j*Tb && s < j*Tb + t5b
        is_on = true;
        break;
    end
end

if is_on
    EF = A_cmd * delta_pulse;
else
    EF = 0;
end
end