function OUT = run_pid_fixed_baseline(T4_des, opts)
%RUN_PID_FIXED_BASELINE Fixed-gain sampled-data PID baseline.
%
%   OUT = RUN_PID_FIXED_BASELINE()
%   OUT = RUN_PID_FIXED_BASELINE(T4_des)
%   OUT = RUN_PID_FIXED_BASELINE(T4_des, opts)
%
% This baseline retains the unchanged 16-state plant, burst-averaged
% actuator, sampled measurements and PID updates, saturation/rate limits,
% derivative-on-measurement, and anti-windup. It excludes gain adaptation,
% band-lock, robust scenario-CVaR updates, and bias correction.

    if nargin < 1 || isempty(T4_des)
        T4_des = 25;
    end
    if nargin < 2 || isempty(opts)
        opts = struct();
    end

    P = get_defaults(opts);
    P.T4_des = T4_des;
    rng(P.rng_seed, 'twister');

    t_end_min = P.t_end_hr*60;
    tspan = 0:P.dt_min:t_end_min;
    nSteps = numel(tspan);

    y = zeros(numel(P.y0), nSteps);
    y(:,1) = P.y0(:);

    next_meas_time = P.T_measure_min;
    next_ctrl_time = P.Ts_min;

    I_state = 0;
    d_f = 0;

    z_used = y(P.idx_T4_ext,1);
    z_prev = z_used;

    EF_cmd = P.EF0;
    A_cmd = ef_to_amp(EF_cmd, P);

    t_meas = 0;
    T4_meas = z_used;

    ctrl_time_hr = 0;
    e_log = P.T4_des-z_used;
    I_log = I_state;
    d_log = d_f;

    EF_unsat_log = EF_cmd;
    EF_cmd_log = EF_cmd;
    A_cmd_log = A_cmd;
    K_hist = [P.Kp,P.Ki,P.Kd];

    EF_sat_log = is_at_bound(EF_cmd,P.EF_max);
    A_sat_log = is_at_bound(A_cmd,P.A_max);

    for k = 1:(nSteps-1)
        tk = tspan(k);
        tk1 = tspan(k+1);

        EF_now = burst_avg_EF(tk,A_cmd,P);

        y(:,k+1) = rk4_step( ...
            @(tt,yy) plant_rhs(tt,yy,EF_now,P), ...
            tk,y(:,k),P.dt_min);
        y(:,k+1) = max(y(:,k+1),0);

        if tk1 >= next_meas_time-1e-12
            z_true = y(P.idx_T4_ext,k+1);
            z_used = max(z_true+P.meas_sigma*randn(),0);

            t_meas(end+1,1) = tk1/60; %#ok<AGROW>
            T4_meas(end+1,1) = z_used; %#ok<AGROW>
            next_meas_time = next_meas_time+P.T_measure_min;
        end

        if tk1 >= next_ctrl_time-1e-12
            e_m = P.T4_des-z_used;

            d_m = (z_used-z_prev)/P.Ts_min;
            d_f = (1-P.alpha_d)*d_f+P.alpha_d*d_m;

            I_state = (1-P.I_leak)*I_state+e_m*P.Ts_min;
            I_state = clamp(I_state,-P.I_max,P.I_max);

            D_m = -P.Kd*d_f;
            EF_unsat = P.Kp*e_m+P.Ki*I_state+D_m;

            EF_rlim = rate_limit(EF_unsat,EF_cmd,P.dEF_max);
            EF_new = clamp(EF_rlim,P.EF_min,P.EF_max);

            I_state = I_state+ ...
                P.Kb_aw*(EF_new-EF_unsat)*P.Ts_min;
            I_state = clamp(I_state,-P.I_max,P.I_max);

            A_new = ef_to_amp(EF_new,P);
            A_new = rate_limit(A_new,A_cmd,P.dA_max);
            A_new = clamp(A_new,P.A_min,P.A_max);

            EF_cmd = EF_new;
            A_cmd = A_new;
            z_prev = z_used;

            ctrl_time_hr(end+1,1) = tk1/60; %#ok<AGROW>
            e_log(end+1,1) = e_m; %#ok<AGROW>
            I_log(end+1,1) = I_state; %#ok<AGROW>
            d_log(end+1,1) = d_f; %#ok<AGROW>
            EF_unsat_log(end+1,1) = EF_unsat; %#ok<AGROW>
            EF_cmd_log(end+1,1) = EF_cmd; %#ok<AGROW>
            A_cmd_log(end+1,1) = A_cmd; %#ok<AGROW>
            K_hist(end+1,:) = [P.Kp,P.Ki,P.Kd]; %#ok<AGROW>
            EF_sat_log(end+1,1) = ...
                is_at_bound(EF_cmd,P.EF_max); %#ok<AGROW>
            A_sat_log(end+1,1) = ...
                is_at_bound(A_cmd,P.A_max); %#ok<AGROW>

            next_ctrl_time = next_ctrl_time+P.Ts_min;
        end
    end

    OUT = struct();
    OUT.time_hr = tspan(:)/60;
    OUT.y = y.';
    OUT.T4_true = y(P.idx_T4_ext,:).';

    OUT.t_meas = t_meas;
    OUT.T4_meas = T4_meas;

    OUT.ctrl_time_hr = ctrl_time_hr;
    OUT.sample_times = ctrl_time_hr*60;

    OUT.e_meas = e_log;
    OUT.I_log = I_log;
    OUT.d_log = d_log;

    OUT.EF_unsat = EF_unsat_log;
    OUT.EF_cmd = EF_cmd_log;
    OUT.A_cmd = A_cmd_log;
    OUT.K_hist = K_hist;

    OUT.EF_saturated = logical(EF_sat_log);
    OUT.A_saturated = logical(A_sat_log);
    OUT.EF_saturation_fraction = mean(OUT.EF_saturated);
    OUT.A_saturation_fraction = mean(OUT.A_saturated);

    OUT.setpoint = P.T4_des;
    OUT.T4_des = P.T4_des;
    OUT.masterSeed = P.rng_seed;
    OUT.P = P;
end


function P = get_defaults(opts)
    P.t_end_hr = get_opt(opts,'t_end_hr',80);
    P.dt_min = get_opt(opts,'dt_min',0.1/60);
    P.Ts_min = get_opt(opts,'Ts_min',120);
    P.T_measure_min = get_opt(opts,'T_measure_min',20);
    P.rng_seed = get_opt(opts,'rng_seed',1);

    P.idx_T4_ext = get_opt(opts,'idx_T4_ext',4);
    P.y0 = get_opt(opts,'y0',default_y0());

    P.N = get_opt(opts,'N',10);
    P.alpha1 = get_opt(opts,'alpha1',3.308);
    P.K = get_opt(opts,'K',1.195);
    P.n = get_opt(opts,'n',2);
    P.gamma1 = get_opt(opts,'gamma1',0.0296);
    P.a = get_opt(opts,'a',0.001);
    P.alpha = get_opt(opts,'alpha',168.7624);
    P.gamma_NKX21 = get_opt(opts,'gamma_NKX21',0.01);
    P.alpha_NKX21_offset = ...
        get_opt(opts,'alpha_NKX21_offset',0.25);

    P.kappa = get_opt(opts,'kappa',1.0);
    P.K2 = get_opt(opts,'K2',1000);
    P.alpha_I = get_opt(opts,'alpha_I',1.0);
    P.gamma_I = get_opt(opts,'gamma_I',0.004);
    P.alpha_TG = get_opt(opts,'alpha_TG',1.0);
    P.gamma_Tg = get_opt(opts,'gamma_Tg',0.04);
    P.gammaint_T4 = get_opt(opts,'gammaint_T4',0.1504);
    P.M_fun = get_opt(opts,'M_fun',0.15);
    P.gammaext_T4 = get_opt(opts,'gammaext_T4',0.01);

    P.Kp = get_opt(opts,'Kp',0.015);
    P.Ki = get_opt(opts,'Ki',1e-5);
    P.Kd = get_opt(opts,'Kd',0);

    P.meas_sigma = get_opt(opts,'meas_sigma',0);
    P.alpha_d = get_opt(opts,'alpha_d',0.25);
    P.I_leak = get_opt(opts,'I_leak',0);
    P.I_max = get_opt(opts,'I_max',1e4);
    P.Kb_aw = get_opt(opts,'Kb_aw',1e-3);

    P.EF_min = get_opt(opts,'EF_min',0);
    P.EF_max = get_opt(opts,'EF_max',100);
    P.EF0 = get_opt(opts,'EF0',0);

    P.A_min = get_opt(opts,'A_min',0);
    P.A_max = get_opt(opts,'A_max',0.10);
    P.thr = get_opt(opts,'thr',1e-4);
    P.kA = get_opt(opts,'kA',0.20);
    P.dEF_max = get_opt(opts,'dEF_max',10);
    P.dA_max = get_opt(opts,'dA_max',0.01);

    P.t1 = get_opt(opts,'t1',500e-6/60);
    P.t2 = get_opt(opts,'t2',100e-6/60);
    P.t3 = get_opt(opts,'t3',500e-6/60);
    P.t4 = get_opt(opts,'t4',9.5e-4);
    P.np = get_opt(opts,'np',500);
    P.tgap = get_opt(opts,'tgap',0);
    P.t6_min = get_opt(opts,'t6_min',20);

    validate_defaults(P);
end


function validate_defaults(P)
    if P.t_end_hr <= 0 || P.dt_min <= 0 || ...
            P.Ts_min <= 0 || P.T_measure_min <= 0
        error('All simulation time settings must be positive.');
    end
    if P.EF_max <= P.EF_min
        error('EF bounds must satisfy EF_min < EF_max.');
    end
    if P.A_max <= P.A_min
        error('Amplitude bounds must satisfy A_min < A_max.');
    end
    if any([P.Kp,P.Ki,P.Kd] < 0)
        error('Fixed PID gains must be nonnegative.');
    end
end


function y0 = default_y0()
    y0 = zeros(16,1);
    y0(1:4) = [4;4;4;4];
end


function val = get_opt(S,name,default)
    if isfield(S,name) && ~isempty(S.(name))
        val = S.(name);
    else
        val = default;
    end
end


function dydt = plant_rhs(t,y,EF_now,P)
    dydt = ode_constant_coeff_model( ...
        t,y,@(tt) EF_now,P);
end


function y1 = rk4_step(f,t,y,h)
    k1 = f(t,y);
    k2 = f(t+0.5*h,y+0.5*h*k1);
    k3 = f(t+0.5*h,y+0.5*h*k2);
    k4 = f(t+h,y+h*k3);
    y1 = y+(h/6)*(k1+2*k2+2*k3+k4);
end


function out = clamp(x,lo,hi)
    out = min(max(x,lo),hi);
end


function uNew = rate_limit(uDes,uOld,duMax)
    du = clamp(uDes-uOld,-duMax,duMax);
    uNew = uOld+du;
end


function tf = is_at_bound(x,upperBound)
    tol = max(1e-10,1e-8*max(1,abs(upperBound)));
    tf = abs(x-upperBound) <= tol;
end


function A = ef_to_amp(EF,P)
    if EF < P.thr
        A = 0;
    else
        A = clamp(P.kA*(EF-P.thr),P.A_min,P.A_max);
    end
end


function EF = burst_avg_EF(t_min,A_cmd,P)
    Ts = P.Ts_min;
    s = mod(t_min,Ts);

    Tp = P.t1+P.t2+P.t3+P.t4;
    deltaPulse = (P.t1+P.t3)/Tp;

    t5b = P.np*Tp;
    Tb = t5b+P.tgap;
    nb = max(0,floor((Ts-P.t6_min)/Tb));

    isOn = false;
    for j = 0:(nb-1)
        if s >= j*Tb && s < j*Tb+t5b
            isOn = true;
            break;
        end
    end

    if isOn
        EF = A_cmd*deltaPulse;
    else
        EF = 0;
    end
end
