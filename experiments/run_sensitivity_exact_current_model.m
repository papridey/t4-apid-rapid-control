function OUT = run_sensitivity_exact_current_model(opts)
%RUN_SENSITIVITY_EXACT_CURRENT_MODEL
% Open-loop sensitivity analysis tied to the CURRENT production plant files:
%
%   get_constant_plant_params.m
%   ode_constant_coeff_model.m
%   ef_avg_burst_in_window.m
%
% The routine first performs exact-model consistency checks.  It then runs:
%
%   1) local OAT parameter sensitivity:     0.9, 1.0, 1.1 x nominal
%   2) broad OAT stress test:               0.5, 0.75, 1, 1.5, 2 x nominal
%   3) mean-delay sweep at fixed N = 10
%   4) delay-shape sweep in N at fixed N/a
%   5) structural downstream Hill-exponent sweep around the hard-coded h = 3
%   6) optional local trajectory-sensitivity Jacobian + SVD
%
% IMPORTANT MODEL-MAPPING NOTE
% The CURRENT production files use the following code-field/manuscript-name
% correspondences:
%
%   P.gamma_NKX21        <-> beta
%   P.alpha_NKX21_offset <-> c
%   P.M_fun              <-> omega
%
% The downstream Hill exponent h = 3 is hard-coded in
% ode_constant_coeff_model.m and is therefore NOT included among the
% ordinary parameter OAT sweeps.  It is treated separately as a structural
% model assumption.
%
% Likewise, N is not varied through ode_constant_coeff_model.m because that
% production file explicitly stores the ten delay states in y(6:15).  The
% N-shape and h-structure sweeps use a generalized RHS that is first checked
% numerically against the production RHS at N = 10 and h = 3.
%
% INPUT AVERAGING
% The production input ef_avg_burst_in_window.m returns either
%
%       EF = A*pulseDuty
%
% during a burst or zero otherwise.  For the long sensitivity runs, the
% fast burst cycle is averaged through the promoter Hill nonlinearity:
%
%   d_on = burstDuty * Hill(A*pulseDuty;K,n).
%
% An equivalent constant EF value is then obtained by inverting the same
% Hill function, so the CURRENT ode_constant_coeff_model.m is still used
% directly in all ordinary parameter and mean-delay sweeps.
%
% OUTPUTS
% Results are written to an Excel workbook and MAT file.  The workbook
% contains:
%
%   Consistency    exact-model checks
%   Config         run configuration and nominal parameters
%   Nominal        nominal metrics
%   Local_OAT      +/-10% one-at-a-time results
%   Wide_OAT       0.5x--2x one-at-a-time results
%   Delay          mean-delay sweep, N fixed
%   N_shape        delay-chain shape sweep, mean N/a fixed
%   h_struct       structural h sweep
%   SVD            singular values of local trajectory-sensitivity matrix
%   V_modes        right singular vectors (parameter directions)
%   SensCorr       pairwise correlation of sensitivity columns
%
% USAGE
% -----
%   opts = struct();
%   opts.output_dir = fullfile(projectRoot,'results',...
%       'sensitivity_exact_current_model');
%   OUT = run_sensitivity_exact_current_model(opts);
%
% The three current production functions must already be on the MATLAB path.
%
% -------------------------------------------------------------------------

if nargin < 1 || isempty(opts)
    opts = struct();
end

opts = apply_defaults(opts);

%% ------------------------------------------------------------------------
%  Locate and verify CURRENT production files
% -------------------------------------------------------------------------
requiredFunctions = { ...
    'get_constant_plant_params', ...
    'ode_constant_coeff_model', ...
    'ef_avg_burst_in_window'};

for k = 1:numel(requiredFunctions)
    f = which(requiredFunctions{k});
    if isempty(f)
        error(['Required current plant function was not found on the MATLAB path:\n' ...
               '  %s.m'], requiredFunctions{k});
    end
end

if ~exist(opts.output_dir,'dir')
    mkdir(opts.output_dir);
end

P0 = get_constant_plant_params();
S  = current_schedule();

% These are the continuously varied non-delay parameters that actually
% exist independently in the CURRENT production parameter structure.
plist = parameter_list();

% Verify required fields before doing any simulation.
requiredFields = unique([{plist.code_field}, ...
    {'N','a','gamma_NKX21','alpha_NKX21_offset','M_fun'}]);

for k = 1:numel(requiredFields)
    if ~isfield(P0,requiredFields{k})
        error('Current parameter structure P is missing field "%s".', ...
            requiredFields{k});
    end
end

if P0.N ~= 10
    error(['The current ode_constant_coeff_model.m explicitly uses y(6:15), ' ...
           'so the production implementation requires P.N = 10.  Current P.N = %g.'], ...
           P0.N);
end

fprintf('\n============================================================\n');
fprintf('Sensitivity analysis tied to CURRENT production plant files\n');
fprintf('============================================================\n');
fprintf('Parameter file : %s\n', which('get_constant_plant_params'));
fprintf('ODE file       : %s\n', which('ode_constant_coeff_model'));
fprintf('EF input file  : %s\n', which('ef_avg_burst_in_window'));
fprintf('Output folder  : %s\n', opts.output_dir);

%% ------------------------------------------------------------------------
%  Exact-model consistency checks
% -------------------------------------------------------------------------
Consistency = exact_model_consistency(P0,S,opts);

disp(Consistency);

if opts.stop_on_consistency_fail && ~all(Consistency.pass)
    error(['At least one exact-model consistency check failed.  ' ...
           'Sensitivity sweeps were not started.']);
end

% Optional first-stage audit: stop after the exact-model checks.
% This is useful before committing to the full sensitivity sweep.
if opts.consistency_only
    Config = make_config_table(P0,S,opts,plist,P0.N/P0.a);

    xlsxFile = fullfile(opts.output_dir,'sensitivity_consistency_check.xlsx');
    if exist(xlsxFile,'file')
        delete(xlsxFile);
    end
    writetable(Consistency,xlsxFile,'Sheet','Consistency');
    writetable(Config,xlsxFile,'Sheet','Config');

    OUT = struct();
    OUT.options = opts;
    OUT.nominal_params = P0;
    OUT.schedule = S;
    OUT.parameter_list = plist;
    OUT.Consistency = Consistency;
    OUT.Config = Config;

    matFile = fullfile(opts.output_dir,'sensitivity_consistency_check.mat');
    save(matFile,'OUT','-v7.3');

    fprintf('\nConsistency-only audit complete.\n');
    fprintf('Workbook: %s\n',xlsxFile);
    fprintf('MAT file: %s\n',matFile);
    return;
end

%% ------------------------------------------------------------------------
%  Nominal simulation using CURRENT ODE
% -------------------------------------------------------------------------
fprintf('\n--- Nominal current-model run ---\n');

[tNom,yNom] = simulate_current_averaged(P0,S,opts,opts.sim_hours);
Mnom = compute_metrics(tNom,yNom,S,opts);

Nominal = metric_table(Mnom);
Nominal.parameter = "nominal";
Nominal.code_field = "nominal";
Nominal.multiplier = 1;
Nominal.value = NaN;
Nominal = movevars(Nominal, ...
    {'parameter','code_field','multiplier','value'},'Before',1);

fprintf(['Nominal: basal = %.6g, ss = %.6g, rise = %.4g h, ' ...
         'ripple = %.6g (%.3f%%), period = %.3f min\n'], ...
    Mnom.T4_basal,Mnom.T4_ss,Mnom.rise_h, ...
    Mnom.ripple,Mnom.ripple_pct,Mnom.ripple_period_min);

%% ------------------------------------------------------------------------
%  Local OAT: +/-10%
% -------------------------------------------------------------------------
fprintf('\n--- Local OAT sensitivity: +/-10%% ---\n');

Local_OAT = run_oat_sweep(P0,S,opts,plist, ...
    opts.local_multipliers,Mnom,"local");

%% ------------------------------------------------------------------------
%  Broad OAT: 0.5x--2x
% -------------------------------------------------------------------------
fprintf('\n--- Broad OAT stress test: 0.5x--2x ---\n');

Wide_OAT = run_oat_sweep(P0,S,opts,plist, ...
    opts.wide_multipliers,Mnom,"wide");

%% ------------------------------------------------------------------------
%  Mean-delay sweep: N fixed, vary N/a through a
% -------------------------------------------------------------------------
fprintf('\n--- Mean-delay sweep with N fixed at %d ---\n',P0.N);

tau0_s = P0.N/P0.a;
rows = cell(numel(opts.delay_multipliers),1);

for j = 1:numel(opts.delay_multipliers)
    mult = opts.delay_multipliers(j);

    P = P0;
    tau_s = mult*tau0_s;
    P.a = P0.N/tau_s;

    [t,y] = simulate_current_averaged(P,S,opts,opts.sim_hours);
    M = compute_metrics(t,y,S,opts);

    T = metric_table(M);
    T.multiplier = mult;
    T.N = P0.N;
    T.a = P.a;
    T.mean_delay_h = tau_s/3600;
    T = movevars(T,{'multiplier','N','a','mean_delay_h'},'Before',1);
    T = add_metric_changes(T,Mnom);

    rows{j} = T;

    fprintf('  tau/tau0=%5.2f  mean delay=%7.4f h  T4ss=%9.5g  rise=%8.4g h\n', ...
        mult,tau_s/3600,M.T4_ss,M.rise_h);
end

Delay = vertcat(rows{:});

%% ------------------------------------------------------------------------
%  Delay-shape sweep: vary N at fixed mean N/a
%  Uses generalized RHS, cross-checked above at N=10,h=3.
% -------------------------------------------------------------------------
fprintf('\n--- Delay-shape sweep: N varied at fixed mean delay ---\n');

rows = cell(numel(opts.N_list),1);

for j = 1:numel(opts.N_list)
    N = opts.N_list(j);

    P = P0;
    P.N = N;
    P.a = N/tau0_s;

    [t,y] = simulate_generalized_averaged(P,N,3,S,opts,opts.sim_hours);
    M = compute_metrics(t,y,S,opts);

    T = metric_table(M);
    T.N = N;
    T.a = P.a;
    T.mean_delay_h = tau0_s/3600;
    T = movevars(T,{'N','a','mean_delay_h'},'Before',1);
    T = add_metric_changes(T,Mnom);

    rows{j} = T;

    fprintf('  N=%3d  a=%9.4g  T4ss=%9.5g  rise=%8.4g h\n', ...
        N,P.a,M.T4_ss,M.rise_h);
end

N_shape = vertcat(rows{:});

%% ------------------------------------------------------------------------
%  Structural downstream Hill-exponent sweep
%  h=3 is hard-coded in the production ODE, so this is NOT labeled ordinary
%  parameter sensitivity.  It is a structural assumption check.
% -------------------------------------------------------------------------
fprintf('\n--- Structural downstream Hill-exponent sweep ---\n');

rows = cell(numel(opts.h_list),1);

for j = 1:numel(opts.h_list)
    h = opts.h_list(j);

    [t,y] = simulate_generalized_averaged(P0,P0.N,h,S,opts,opts.sim_hours);
    M = compute_metrics(t,y,S,opts);

    T = metric_table(M);
    T.h = h;
    T = movevars(T,'h','Before',1);
    T = add_metric_changes(T,Mnom);

    rows{j} = T;

    fprintf('  h=%5.2f  T4ss=%9.5g  rise=%8.4g h  ripple=%9.5g\n', ...
        h,M.T4_ss,M.rise_h,M.ripple);
end

h_struct = vertcat(rows{:});

%% ------------------------------------------------------------------------
%  Optional local trajectory sensitivity Jacobian and SVD
% -------------------------------------------------------------------------
SVD_table = table();
V_modes   = table();
SensCorr  = table();
trajectory_sensitivity = [];
trajectory_time_h = [];

if opts.compute_trajectory_jacobian
    fprintf('\n--- Local trajectory-sensitivity Jacobian and SVD ---\n');

    delta = opts.jacobian_delta;
    nP = numel(plist);
    nT = numel(tNom);

    trajectory_sensitivity = zeros(nT,nP);
    trajectory_time_h = tNom(:)/60;

    yScale = max(abs(yNom));
    if ~isfinite(yScale) || yScale <= eps
        yScale = 1;
    end

    for ip = 1:nP
        field = plist(ip).code_field;

        Pminus = P0;
        Pplus  = P0;

        Pminus.(field) = P0.(field)*(1-delta);
        Pplus.(field)  = P0.(field)*(1+delta);

        [tm,ym] = simulate_current_averaged(Pminus,S,opts,opts.sim_hours);
        [tp,yp] = simulate_current_averaged(Pplus, S,opts,opts.sim_hours);

        if numel(tm) ~= nT || numel(tp) ~= nT || ...
                max(abs(tm-tNom)) > 1e-10 || max(abs(tp-tNom)) > 1e-10
            error('Trajectory grids do not match for Jacobian parameter %s.',field);
        end

        % Dimensionless local sensitivity to log(parameter):
        %
        % (theta/Yscale) dY/dtheta
        % ~= [Y(theta(1+d))-Y(theta(1-d))]/(2*d*Yscale)
        trajectory_sensitivity(:,ip) = ...
            (yp-ym)/(2*delta*yScale);

        fprintf('  Jacobian column %2d/%2d: %s\n',ip,nP,field);
    end

    [~,Sigma,V] = svd(trajectory_sensitivity,'econ');
    s = diag(Sigma);

    if isempty(s) || s(1) <= eps
        rels = zeros(size(s));
    else
        rels = s/s(1);
    end

    mode = (1:numel(s))';
    SVD_table = table(mode,s,rels, ...
        'VariableNames',{'mode','singular_value','relative_singular_value'});

    SVD_table.rank_rel_1e2 = repmat(sum(rels >= 1e-2),height(SVD_table),1);
    SVD_table.rank_rel_1e3 = repmat(sum(rels >= 1e-3),height(SVD_table),1);
    SVD_table.rank_rel_1e4 = repmat(sum(rels >= 1e-4),height(SVD_table),1);

    % Right singular vectors describe parameter-space directions.
    nModesSave = min(opts.num_svd_modes_to_save,size(V,2));
    V_modes = table(string({plist.manuscript_name})', ...
                    string({plist.code_field})', ...
                    'VariableNames',{'parameter','code_field'});

    for k = 1:nModesSave
        V_modes.(sprintf('mode_%02d',k)) = V(:,k);
    end

    C = corrcoef(trajectory_sensitivity);
    SensCorr = array2table(C, ...
        'VariableNames', matlab.lang.makeValidName({plist.code_field}), ...
        'RowNames', matlab.lang.makeValidName({plist.code_field}));

    fprintf('  Effective rank at relative singular-value thresholds:\n');
    fprintf('    1e-2 : %d / %d\n',sum(rels>=1e-2),nP);
    fprintf('    1e-3 : %d / %d\n',sum(rels>=1e-3),nP);
    fprintf('    1e-4 : %d / %d\n',sum(rels>=1e-4),nP);
    fprintf(['  NOTE: this is a numerical sensitivity rank for this experiment;\n' ...
             '        it is NOT a structural-identifiability proof.\n']);
end

%% ------------------------------------------------------------------------
%  Configuration table
% -------------------------------------------------------------------------
Config = make_config_table(P0,S,opts,plist,tau0_s);

%% ------------------------------------------------------------------------
%  Save Excel workbook
% -------------------------------------------------------------------------
xlsxFile = fullfile(opts.output_dir,opts.xlsx_name);

if exist(xlsxFile,'file')
    delete(xlsxFile);
end

writetable(Consistency,xlsxFile,'Sheet','Consistency');
writetable(Config,     xlsxFile,'Sheet','Config');
writetable(Nominal,    xlsxFile,'Sheet','Nominal');
writetable(Local_OAT,  xlsxFile,'Sheet','Local_OAT');
writetable(Wide_OAT,   xlsxFile,'Sheet','Wide_OAT');
writetable(Delay,      xlsxFile,'Sheet','Delay');
writetable(N_shape,    xlsxFile,'Sheet','N_shape');
writetable(h_struct,   xlsxFile,'Sheet','h_struct');

if opts.compute_trajectory_jacobian
    writetable(SVD_table,xlsxFile,'Sheet','SVD');
    writetable(V_modes,  xlsxFile,'Sheet','V_modes');

    % Write row names explicitly for portability.
    SensCorrOut = addvars(SensCorr, ...
        string(SensCorr.Properties.RowNames), ...
        'Before',1,'NewVariableNames','parameter');
    SensCorrOut.Properties.RowNames = {};
    writetable(SensCorrOut,xlsxFile,'Sheet','SensCorr');
end

%% ------------------------------------------------------------------------
%  Package outputs and save MAT file
% -------------------------------------------------------------------------
OUT = struct();
OUT.options = opts;
OUT.nominal_params = P0;
OUT.schedule = S;
OUT.parameter_list = plist;
OUT.Consistency = Consistency;
OUT.Config = Config;
OUT.Nominal = Nominal;
OUT.Local_OAT = Local_OAT;
OUT.Wide_OAT = Wide_OAT;
OUT.Delay = Delay;
OUT.N_shape = N_shape;
OUT.h_struct = h_struct;
OUT.SVD = SVD_table;
OUT.V_modes = V_modes;
OUT.SensCorr = SensCorr;
OUT.trajectory_time_h = trajectory_time_h;
OUT.trajectory_sensitivity = trajectory_sensitivity;
OUT.nominal_time_h = tNom/60;
OUT.nominal_T4ext = yNom;

matFile = fullfile(opts.output_dir,'sensitivity_exact_current_model.mat');
save(matFile,'OUT','-v7.3');

fprintf('\n============================================================\n');
fprintf('Sensitivity analysis complete.\n');
fprintf('Workbook: %s\n',xlsxFile);
fprintf('MAT file: %s\n',matFile);
fprintf('============================================================\n');

end


%% =========================================================================
%  DEFAULTS
% =========================================================================
function opts = apply_defaults(opts)

opts = setdef(opts,'output_dir', ...
    fullfile(pwd,'sensitivity_exact_current_model'));

opts = setdef(opts,'xlsx_name', ...
    'sensitivity_exact_current_model.xlsx');

opts = setdef(opts,'A_fixed',0.012);
opts = setdef(opts,'sim_hours',200);
opts = setdef(opts,'out_dt_min',1);
opts = setdef(opts,'tail_hours',20);

opts = setdef(opts,'local_multipliers',[0.9 1.0 1.1]);
opts = setdef(opts,'wide_multipliers',[0.5 0.75 1.0 1.5 2.0]);

opts = setdef(opts,'delay_multipliers',[0.25 0.5 1 2 4]);
opts = setdef(opts,'N_list',[3 5 10 20 40]);
opts = setdef(opts,'h_list',[2 3 4]);

opts = setdef(opts,'RelTol',1e-8);
opts = setdef(opts,'AbsTol',1e-10);

opts = setdef(opts,'consistency_hours',12);
opts = setdef(opts,'consistency_tol_rhs',1e-11);
opts = setdef(opts,'consistency_tol_input',1e-14);
opts = setdef(opts,'consistency_tol_traj',1e-6);
opts = setdef(opts,'stop_on_consistency_fail',true);
opts = setdef(opts,'consistency_only',false);

opts = setdef(opts,'compute_trajectory_jacobian',true);
opts = setdef(opts,'jacobian_delta',0.01);
opts = setdef(opts,'num_svd_modes_to_save',8);

end


%% =========================================================================
%  PARAMETER LIST
% =========================================================================
function plist = parameter_list()
% Only independently stored, continuously varied, non-delay parameters from
% get_constant_plant_params.m are included here.

fields = { ...
    'alpha1', ...
    'K', ...
    'n', ...
    'gamma1', ...
    'alpha', ...
    'gamma_NKX21', ...
    'alpha_NKX21_offset', ...
    'kappa', ...
    'K2', ...
    'alpha_I', ...
    'gamma_I', ...
    'alpha_TG', ...
    'gamma_Tg', ...
    'gammaint_T4', ...
    'M_fun', ...
    'gammaext_T4'};

names = { ...
    '\alpha_1', ...
    'K', ...
    'n', ...
    '\gamma_1', ...
    '\alpha', ...
    '\beta', ...
    'c', ...
    '\kappa', ...
    'K_2', ...
    '\alpha_I', ...
    '\gamma_I', ...
    '\alpha_{Tg}', ...
    '\gamma_{Tg}', ...
    '\gamma_{T4}^{int}', ...
    '\omega', ...
    '\gamma_{T4}^{ext}'};

groups = { ...
    'EF-promoter', ...
    'EF-promoter', ...
    'EF-promoter', ...
    'EF-promoter', ...
    'NKX21', ...
    'NKX21', ...
    'NKX21', ...
    'production node', ...
    'production node', ...
    'core T4', ...
    'core T4', ...
    'core T4', ...
    'core T4', ...
    'core T4', ...
    'core T4', ...
    'core T4'};

n = numel(fields);
plist = repmat(struct( ...
    'code_field','', ...
    'manuscript_name','', ...
    'group',''),n,1);

for k = 1:n
    plist(k).code_field = fields{k};
    plist(k).manuscript_name = names{k};
    plist(k).group = groups{k};
end

end


%% =========================================================================
%  CURRENT STIMULATION SCHEDULE
% =========================================================================
function S = current_schedule()
% Matches ef_avg_burst_in_window.m and the current APID timing convention.

S.T_cycle_min = 120;
S.active_fraction = 0.60;
S.T_on_min = S.active_fraction*S.T_cycle_min;

t1 = 500e-6;   % seconds
t2 = 100e-6;
t3 = 500e-6;
t4 = 0.00095;

S.Tp_s = t1+t2+t3+t4;
S.pulseDuty = (t1+t3)/S.Tp_s;

S.np = 500;
S.t5_s = S.np*S.Tp_s;
S.t6_s = 0.5;
S.Tb_s = S.t5_s+S.t6_s;
S.burstDuty = S.t5_s/S.Tb_s;

S.t5_min = S.t5_s/60;
S.t6_min = S.t6_s/60;

end


%% =========================================================================
%  EXACT-MODEL CONSISTENCY CHECKS
% =========================================================================
function T = exact_model_consistency(P0,S,opts)

check = strings(0,1);
value = zeros(0,1);
tolerance = zeros(0,1);
pass = false(0,1);
note = strings(0,1);

% -------------------------------------------------------------------------
% A. Source/layout check
% -------------------------------------------------------------------------
odePath = which('ode_constant_coeff_model');
src = fileread(odePath);

hasTenStateSlice = ~isempty(regexp(src,'y\s*\(\s*6\s*:\s*15\s*\)','once'));
hasHardcodedH3   = ~isempty(regexp(src,'P\.K2\s*\)\s*\^3','once')) || ...
                   contains(src,'^3');

check(end+1,1) = "production_N_layout";
value(end+1,1) = double(P0.N);
tolerance(end+1,1) = 10;
pass(end+1,1) = (P0.N == 10) && hasTenStateSlice;
note(end+1,1) = "Current ODE uses the ten-state delay slice y(6:15).";

check(end+1,1) = "production_h_structure";
value(end+1,1) = 3;
tolerance(end+1,1) = 3;
pass(end+1,1) = hasHardcodedH3;
note(end+1,1) = "Current downstream Hill exponent is treated as structural h=3.";

% -------------------------------------------------------------------------
% B. Pointwise RHS consistency:
%    generalized N=10,h=3 versus CURRENT ode_constant_coeff_model
% -------------------------------------------------------------------------
rng(123,'twister');

maxAbs = 0;
maxRel = 0;

ybase = basal_state_generalized(P0,10,3);
efVals = [0, 0.25*opts.A_fixed*S.pulseDuty, ...
             0.75*opts.A_fixed*S.pulseDuty, ...
             opts.A_fixed*S.pulseDuty];

for iy = 1:6
    perturb = 1 + 0.05*randn(size(ybase));
    y = max(ybase.*perturb,0);
    y(5:15) = y(5:15) + 0.01*rand(11,1);

    for ie = 1:numel(efVals)
        ef = efVals(ie);
        EFfun = @(tmin) ef + 0*tmin;

        dCurrent_min = ode_constant_coeff_model(0,y,EFfun,P0);
        dGeneral_min = rhs_generalized_min(y,ef,P0,10,3);

        d = dCurrent_min-dGeneral_min;
        maxAbs = max(maxAbs,max(abs(d)));

        denom = max(max(abs(dCurrent_min)),1);
        maxRel = max(maxRel,max(abs(d))/denom);
    end
end

check(end+1,1) = "RHS_current_vs_generalized";
value(end+1,1) = maxAbs;
tolerance(end+1,1) = opts.consistency_tol_rhs;
pass(end+1,1) = maxAbs <= opts.consistency_tol_rhs;
note(end+1,1) = sprintf('max relative discrepancy %.3e',maxRel);

% -------------------------------------------------------------------------
% C. Pointwise exact-input consistency
% -------------------------------------------------------------------------
Ttotal = 3*S.T_cycle_min;
times = [ ...
    0; ...
    S.t5_min/2; ...
    S.t5_min+S.t6_min/2; ...
    S.T_on_min-1e-6; ...
    S.T_on_min+1e-6; ...
    S.T_cycle_min+S.t5_min/2; ...
    2*S.T_cycle_min+S.T_on_min+1e-5];

for k = 1:30
    times(end+1,1) = rand*Ttotal; %#ok<AGROW>
end

maxInputDiff = 0;

for k = 1:numel(times)
    tmin = times(k);

    eCurrent = ef_avg_burst_in_window( ...
        tmin,S.T_on_min,S.T_cycle_min,Ttotal, ...
        S.t5_min,S.t6_min,opts.A_fixed,S.pulseDuty);

    eExpected = exact_schedule_value(tmin,Ttotal,S,opts.A_fixed);

    maxInputDiff = max(maxInputDiff,abs(eCurrent-eExpected));
end

check(end+1,1) = "EF_schedule_pointwise";
value(end+1,1) = maxInputDiff;
tolerance(end+1,1) = opts.consistency_tol_input;
pass(end+1,1) = maxInputDiff <= opts.consistency_tol_input;
note(end+1,1) = "Current EF function matches the stated window/burst schedule.";

% -------------------------------------------------------------------------
% D. Nominal averaged-trajectory consistency:
%    CURRENT ODE versus generalized N=10,h=3
% -------------------------------------------------------------------------
[t1,y1] = simulate_current_averaged(P0,S,opts,opts.consistency_hours);
[t2,y2] = simulate_generalized_averaged(P0,10,3,S,opts,opts.consistency_hours);

if numel(t1) ~= numel(t2) || max(abs(t1-t2)) > 1e-12
    trajRel = Inf;
else
    scale = max(max(abs(y1)),1);
    trajRel = max(abs(y1-y2))/scale;
end

check(end+1,1) = "trajectory_current_vs_generalized";
value(end+1,1) = trajRel;
tolerance(end+1,1) = opts.consistency_tol_traj;
pass(end+1,1) = trajRel <= opts.consistency_tol_traj;
note(end+1,1) = sprintf('%.3g-h averaged-input comparison',opts.consistency_hours);

% -------------------------------------------------------------------------
% E. Report the exact mean promoter drive used during active windows
% -------------------------------------------------------------------------
driveOn = mean_hill_drive(P0,S,opts.A_fixed);

check(end+1,1) = "active_window_mean_Hill_drive";
value(end+1,1) = driveOn;
tolerance(end+1,1) = NaN;
pass(end+1,1) = true;
note(end+1,1) = "burstDuty * Hill(A*pulseDuty;K,n)";

T = table(check,value,tolerance,pass,note);

end


%% =========================================================================
%  ORDINARY OAT SWEEP USING CURRENT PRODUCTION ODE
% =========================================================================
function T = run_oat_sweep(P0,S,opts,plist,multipliers,Mnom,label)

rows = cell(numel(plist)*numel(multipliers),1);
ir = 0;

for ip = 1:numel(plist)
    field = plist(ip).code_field;
    pname = plist(ip).manuscript_name;
    grp = plist(ip).group;

    fprintf('  %-22s (%s)\n',field,label);

    for jm = 1:numel(multipliers)
        mult = multipliers(jm);

        P = P0;
        P.(field) = P0.(field)*mult;

        [t,y] = simulate_current_averaged(P,S,opts,opts.sim_hours);
        M = compute_metrics(t,y,S,opts);

        R = metric_table(M);
        R.parameter = string(pname);
        R.code_field = string(field);
        R.group = string(grp);
        R.multiplier = mult;
        R.nominal_value = P0.(field);
        R.value = P.(field);

        R = movevars(R, ...
            {'parameter','code_field','group','multiplier', ...
             'nominal_value','value'},'Before',1);

        R = add_metric_changes(R,Mnom);

        ir = ir+1;
        rows{ir} = R;
    end
end

T = vertcat(rows{:});

end


%% =========================================================================
%  SIMULATE USING CURRENT PRODUCTION ODE WITH BURST-CYCLE-AVERAGED DRIVE
% =========================================================================
function [t_out,y4_out] = simulate_current_averaged(P,S,opts,sim_hours)

if P.N ~= 10
    error(['simulate_current_averaged uses the CURRENT ode_constant_coeff_model, ' ...
           'which requires N=10.']);
end

Tend = sim_hours*60;
dt = opts.out_dt_min;
tgrid = (0:dt:Tend)';

y = basal_state_generalized(P,10,3);

y4_out = nan(size(tgrid));
y4_out(1) = y(4);

odeOpts = odeset( ...
    'RelTol',opts.RelTol, ...
    'AbsTol',opts.AbsTol);

EFon = equivalent_EF_for_mean_hill(P,S,opts.A_fixed);

edges = window_edges(S,Tend);

for k = 1:numel(edges)-1
    ta = edges(k);
    tb = edges(k+1);

    active = mod(ta,S.T_cycle_min) < S.T_on_min-1e-10;

    if active
        EFconst = EFon;
    else
        EFconst = 0;
    end

    EFfun = @(tmin) EFconst + 0*tmin;

    idx = find(tgrid > ta & tgrid <= tb);
    span = unique([ta; tgrid(idx); tb]);

    if numel(span) < 2
        continue;
    end

    if numel(span) == 2
        span = [span(1); mean(span); span(2)];
    end

    [~,Y] = ode15s( ...
        @(tt,yy) ode_constant_coeff_model(tt,yy,EFfun,P), ...
        span,y,odeOpts);

    y = Y(end,:)';

    if ~isempty(idx)
        [tf,loc] = ismember(tgrid(idx),span);
        y4_out(idx(tf)) = Y(loc(tf),4);
    end
end

t_out = tgrid;
y4_out = fillmissing(y4_out,'linear');

end


%% =========================================================================
%  GENERALIZED MODEL FOR N AND h STRUCTURAL SWEEPS
% =========================================================================
function [t_out,y4_out] = simulate_generalized_averaged( ...
    P,N,h,S,opts,sim_hours)

Tend = sim_hours*60;
dt = opts.out_dt_min;
tgrid = (0:dt:Tend)';

y = basal_state_generalized(P,N,h);

y4_out = nan(size(tgrid));
y4_out(1) = y(4);

odeOpts = odeset( ...
    'RelTol',opts.RelTol, ...
    'AbsTol',opts.AbsTol);

EFon = equivalent_EF_for_mean_hill(P,S,opts.A_fixed);

edges = window_edges(S,Tend);

for k = 1:numel(edges)-1
    ta = edges(k);
    tb = edges(k+1);

    active = mod(ta,S.T_cycle_min) < S.T_on_min-1e-10;

    if active
        EFconst = EFon;
    else
        EFconst = 0;
    end

    idx = find(tgrid > ta & tgrid <= tb);
    span = unique([ta; tgrid(idx); tb]);

    if numel(span) < 2
        continue;
    end

    if numel(span) == 2
        span = [span(1); mean(span); span(2)];
    end

    [~,Y] = ode15s( ...
        @(tt,yy) rhs_generalized_min(yy,EFconst,P,N,h), ...
        span,y,odeOpts);

    y = Y(end,:)';

    if ~isempty(idx)
        [tf,loc] = ismember(tgrid(idx),span);
        y4_out(idx(tf)) = Y(loc(tf),4);
    end
end

t_out = tgrid;
y4_out = fillmissing(y4_out,'linear');

end


%% =========================================================================
%  GENERALIZED RHS
%  Algebraically identical to the current production ODE at N=10,h=3.
% =========================================================================
function dydt_min = rhs_generalized_min(y,EF,P,N,h)

x = y(1:4);
uf = y(5);
u = y(6:5+N);
nk = y(6+N);

% EF -> promoter drive
duf = P.alpha1 * EF^P.n/(P.K^P.n+EF^P.n) ...
    - P.gamma1*uf;

% Erlang delay chain
du = zeros(N,1);
du(1) = P.a*(uf-u(1));

for j = 2:N
    du(j) = P.a*(u(j-1)-u(j));
end

% NKX21
dnk = P.alpha*u(end) ...
    - P.gamma_NKX21*nk ...
    + P.alpha_NKX21_offset;

% regulated production node
z = (nk/P.K2)^h;
k1 = P.kappa*z/(1+z);

flux = k1*x(3)*x(1);

% core plant
dx1 = -flux - P.gamma_I*x(1) + P.alpha_I;
dx2 =  flux - P.M_fun*x(2) - P.gammaint_T4*x(2);
dx3 =  P.alpha_TG - flux - P.gamma_Tg*x(3);
dx4 =  P.M_fun*x(2) - P.gammaext_T4*x(4);

dydt_sec = [dx1;dx2;dx3;dx4;duf;du;dnk];
dydt_min = 60*dydt_sec;

end


%% =========================================================================
%  BASAL EQUILIBRIUM OF THE CURRENT EQUATIONS
% =========================================================================
function y0 = basal_state_generalized(P,N,h)

% EF=0 => uf=0 and all delay-chain states=0.
nk = P.alpha_NKX21_offset/P.gamma_NKX21;

z = (nk/P.K2)^h;
k1 = P.kappa*z/(1+z);

% At equilibrium let F=k1*Tg*I.
% I  = (alpha_I-F)/gamma_I
% Tg = (alpha_TG-F)/gamma_Tg
%
% Therefore
% F = k1*(alpha_I-F)*(alpha_TG-F)/(gamma_I*gamma_Tg).

A = k1/(P.gamma_I*P.gamma_Tg);

if A <= eps
    F = 0;
else
    coeff = [ ...
        A, ...
        -(A*(P.alpha_I+P.alpha_TG)+1), ...
        A*P.alpha_I*P.alpha_TG];

    rr = roots(coeff);
    rr = rr(abs(imag(rr)) < 1e-9);
    rr = real(rr);

    upper = min(P.alpha_I,P.alpha_TG);
    rr = rr(rr >= -1e-12 & rr <= upper+1e-12);

    if isempty(rr)
        error('Could not determine a physical basal flux.');
    end

    F = min(max(rr,0));
end

I = (P.alpha_I-F)/P.gamma_I;
Tg = (P.alpha_TG-F)/P.gamma_Tg;
T4int = F/(P.M_fun+P.gammaint_T4);
T4ext = P.M_fun*T4int/P.gammaext_T4;

y0 = zeros(6+N,1);
y0(1) = I;
y0(2) = T4int;
y0(3) = Tg;
y0(4) = T4ext;
y0(5) = 0;
y0(6:5+N) = 0;
y0(6+N) = nk;

end


%% =========================================================================
%  INPUT HELPERS
% =========================================================================
function d = mean_hill_drive(P,S,A)

EFburst = A*S.pulseDuty;
hill = EFburst^P.n/(P.K^P.n+EFburst^P.n);
d = S.burstDuty*hill;

end


function EF = equivalent_EF_for_mean_hill(P,S,A)
% Find EF such that Hill(EF;K,n) equals the exact burst-cycle mean of
% Hill(A*pulseDuty;K,n) during the active part of the 120-min window.

d = mean_hill_drive(P,S,A);

if d <= 0
    EF = 0;
elseif d >= 1
    error('Mean Hill drive must be strictly less than 1.');
else
    EF = P.K*(d/(1-d))^(1/P.n);
end

end


function ef = exact_schedule_value(tmin,Ttotal,S,A)

if tmin < 0 || tmin > Ttotal
    ef = 0;
    return;
end

tau = mod(tmin,S.T_cycle_min);

if tau >= S.T_on_min
    ef = 0;
    return;
end

Tb_min = S.t5_min+S.t6_min;
xi = mod(tau,Tb_min);

ef = A*S.pulseDuty*double(xi < S.t5_min);

end


function edges = window_edges(S,Tend)

nW = ceil(Tend/S.T_cycle_min);
edges = zeros(2*nW+1,1);

for w = 0:nW-1
    edges(2*w+1) = w*S.T_cycle_min;
    edges(2*w+2) = w*S.T_cycle_min+S.T_on_min;
end

edges(end) = nW*S.T_cycle_min;
edges = unique(min(edges,Tend));

end


%% =========================================================================
%  METRICS
% =========================================================================
function M = compute_metrics(tmin,y4,S,opts)

tail = tmin >= tmin(end)-opts.tail_hours*60;

M.T4_basal = y4(1);
M.T4_ss = mean(y4(tail));
M.T4_min = min(y4(tail));
M.T4_max = max(y4(tail));
M.ripple = M.T4_max-M.T4_min;
M.ripple_pct = 100*M.ripple/max(abs(M.T4_ss),eps);

% One-window moving-average envelope for rise-time extraction.
nsm = max(1,round(S.T_cycle_min/opts.out_dt_min));
yenv = movmean(y4,nsm,'Endpoints','shrink');

y0 = yenv(1);
span = M.T4_ss-y0;

if abs(span) < 1e-10
    M.t10_h = NaN;
    M.t90_h = NaN;
    M.rise_h = NaN;
else
    t10 = first_crossing(tmin,yenv,y0+0.10*span,span);
    t90 = first_crossing(tmin,yenv,y0+0.90*span,span);

    M.t10_h = t10/60;
    M.t90_h = t90/60;
    M.rise_h = (t90-t10)/60;
end

% Dominant period in the steady-state tail.
z = y4(tail)-mean(y4(tail));

if numel(z) > 8 && any(abs(z)>1e-12)
    dt = mean(diff(tmin(tail)));
    F = abs(fft(z));
    F(1) = 0;

    nf = floor(numel(F)/2);
    [~,k] = max(F(1:nf));

    if k > 1
        M.ripple_period_min = numel(z)*dt/(k-1);
    else
        M.ripple_period_min = NaN;
    end
else
    M.ripple_period_min = NaN;
end

end


function tc = first_crossing(t,y,level,span)

if span >= 0
    idx = find(y>=level,1,'first');
else
    idx = find(y<=level,1,'first');
end

if isempty(idx)
    tc = NaN;
    return;
end

if idx == 1
    tc = t(1);
    return;
end

y1 = y(idx-1);
y2 = y(idx);

if abs(y2-y1) <= eps
    tc = t(idx);
else
    tc = t(idx-1) + ...
        (level-y1)*(t(idx)-t(idx-1))/(y2-y1);
end

end


function T = metric_table(M)

T = table( ...
    M.T4_basal, ...
    M.T4_ss, ...
    M.t10_h, ...
    M.t90_h, ...
    M.rise_h, ...
    M.ripple, ...
    M.ripple_pct, ...
    M.T4_min, ...
    M.T4_max, ...
    M.ripple_period_min, ...
    'VariableNames',{ ...
    'T4_basal', ...
    'T4_ss', ...
    't10_h', ...
    't90_h', ...
    'rise_h', ...
    'ripple', ...
    'ripple_pct', ...
    'T4_min', ...
    'T4_max', ...
    'ripple_period_min'});

end


function T = add_metric_changes(T,M0)

T.dT4_basal_pct = pct_change(T.T4_basal,M0.T4_basal);
T.dT4_ss_pct = pct_change(T.T4_ss,M0.T4_ss);
T.drise_pct = pct_change(T.rise_h,M0.rise_h);
T.dripple_pct = pct_change(T.ripple,M0.ripple);

end


function p = pct_change(x,x0)

if ~isfinite(x0) || abs(x0) <= eps
    p = NaN;
else
    p = 100*(x-x0)/x0;
end

end


%% =========================================================================
%  CONFIG TABLE
% =========================================================================
function T = make_config_table(P0,S,opts,plist,tau0_s)

key = strings(0,1);
value = strings(0,1);

add('A_fixed',opts.A_fixed);
add('sim_hours',opts.sim_hours);
add('out_dt_min',opts.out_dt_min);
add('tail_hours',opts.tail_hours);
add('T_cycle_min',S.T_cycle_min);
add('T_on_min',S.T_on_min);
add('pulseDuty',S.pulseDuty);
add('burst_on_s',S.t5_s);
add('burst_gap_s',S.t6_s);
add('burstDuty',S.burstDuty);
add('nominal_mean_delay_h',tau0_s/3600);
add('RelTol',opts.RelTol);
add('AbsTol',opts.AbsTol);
add('current_parameter_file',which('get_constant_plant_params'));
add('current_ODE_file',which('ode_constant_coeff_model'));
add('current_EF_file',which('ef_avg_burst_in_window'));

for k = 1:numel(plist)
    add(['nom_' plist(k).code_field],P0.(plist(k).code_field));
end

add('nom_N',P0.N);
add('nom_a',P0.a);
add('structural_h_current',3);

T = table(key,value);

    function add(k,v)
        key(end+1,1) = string(k);
        if isnumeric(v)
            value(end+1,1) = string(sprintf('%.16g',v));
        else
            value(end+1,1) = string(v);
        end
    end

end


%% =========================================================================
%  GENERIC DEFAULT HELPER
% =========================================================================
function s = setdef(s,f,v)

if ~isfield(s,f) || isempty(s.(f))
    s.(f) = v;
end

end
