function TUNE = optimize_fixed_pid_for_setpoint(T4_des, opts)
%OPTIMIZE_FIXED_PID_FOR_SETPOINT Offline fixed-PID tuning at one setpoint.
%
%   TUNE = OPTIMIZE_FIXED_PID_FOR_SETPOINT()
%   TUNE = OPTIMIZE_FIXED_PID_FOR_SETPOINT(T4_des)
%   TUNE = OPTIMIZE_FIXED_PID_FOR_SETPOINT(T4_des, opts)
%
% The objective is the full-horizon counterpart of the APID/RAPID
% one-window predictive cost:
%
%   J = integral [beta_ov*[y-r]_+^2 + [y-r]_-^2] dt
%       + rhoA*sum_m A_m^2.
%
% beta_ov and rhoA are read directly from rapid_controller_defaults.m
% unless explicitly supplied through opts.beta_ov and opts.rhoA.
%
% Bounds are imposed in logarithmic gain coordinates and optimized using
% multistart fminsearch. Every objective evaluation is appended to CSV,
% and every completed start is checkpointed.
%
% PAPER RUN:
%   clear; clc; close all;
%   opts = struct();
%   opts.tune_horizon_hr = 80;
%   opts.training_seeds = 1;
%   opts.n_starts = 5;
%   opts.max_fun_evals = 80;
%   opts.max_iter = 60;
%   opts.force_retune = true;
%   TUNE = optimize_fixed_pid_for_setpoint(25, opts);
%
% SMOKE TEST:
%   clear; clc; close all;
%   opts = struct();
%   opts.tune_horizon_hr = 4;
%   opts.training_seeds = 1;
%   opts.n_starts = 1;
%   opts.max_fun_evals = 8;
%   opts.max_iter = 5;
%   opts.force_retune = true;
%   TUNE_TEST = optimize_fixed_pid_for_setpoint(25, opts);

    if nargin < 1 || isempty(T4_des)
        T4_des = 25;
    end
    if nargin < 2 || isempty(opts)
        opts = struct();
    end

    thisFile = mfilename('fullpath');
    thisDir = fileparts(thisFile);
    projectRoot = locate_project_root(thisDir);
    add_project_paths(projectRoot,thisDir);

    opts = apply_defaults(opts,projectRoot,T4_des);
    [opts.beta_ov,opts.rhoA] = ...
        resolve_paper_cost_parameters(T4_des,opts);
    validate_options(opts);

    require_file('run_pid_fixed_baseline');
    require_file('ode_constant_coeff_model');

    if ~exist(opts.output_dir,'dir')
        mkdir(opts.output_dir);
    end

    setpointTag = numeric_file_tag(T4_des);

    resultMat = fullfile(opts.output_dir, ...
        sprintf('optimized_fixed_pid_T4_%s.mat',setpointTag));
    resultCsv = fullfile(opts.output_dir, ...
        sprintf('optimized_fixed_pid_T4_%s.csv',setpointTag));
    evaluationCsv = fullfile(opts.output_dir, ...
        'fixed_pid_tuning_evaluations.csv');
    startsCsv = fullfile(opts.output_dir, ...
        'fixed_pid_tuning_starts.csv');
    checkpointMat = fullfile(opts.output_dir, ...
        'fixed_pid_tuning_checkpoint.mat');

    if exist(resultMat,'file') && ~opts.force_retune
        loaded = load(resultMat,'TUNE');
        if isfield(loaded,'TUNE')
            TUNE = loaded.TUNE;
            fprintf('\nReusing previously optimized gains:\n');
            print_final_result(TUNE,resultMat);
            return;
        end
    end

    if opts.force_retune
        archive_existing_file(resultMat);
        archive_existing_file(resultCsv);
        archive_existing_file(checkpointMat);
    end
    archive_existing_file(evaluationCsv);
    archive_existing_file(startsCsv);

    starts = make_start_points(opts);
    nStarts = size(starts,1);

    startNumber = (1:nStarts)';
    Kp_start = starts(:,1);
    Ki_start = starts(:,2);
    Kd_start = starts(:,3);

    Kp_final = nan(nStarts,1);
    Ki_final = nan(nStarts,1);
    Kd_final = nan(nStarts,1);
    Objective = nan(nStarts,1);
    TrackingCost = nan(nStarts,1);
    AmplitudeCost = nan(nStarts,1);
    ASaturationPct = nan(nStarts,1);
    ExitFlag = nan(nStarts,1);
    Iterations = nan(nStarts,1);
    FunctionEvaluations = nan(nStarts,1);
    SuccessfulEvaluations = zeros(nStarts,1);
    FailedEvaluations = zeros(nStarts,1);

    evalCounter = 0;
    activeStart = 0;
    totalSuccessfulEvaluations = 0;
    totalFailedEvaluations = 0;

    fprintf('\n============================================================\n');
    fprintf('Fixed PID tuning at T4_des = %.6g\n',T4_des);
    fprintf('Training horizon: %.3g h\n',opts.tune_horizon_hr);
    fprintf('Training seeds: %s\n',mat2str(opts.training_seeds));
    fprintf('Gain lower bounds: %s\n',mat2str(opts.gain_lb));
    fprintf('Gain upper bounds: %s\n',mat2str(opts.gain_ub));
    fprintf('beta_ov = %.12g\n',opts.beta_ov);
    fprintf('rhoA = %.12g\n',opts.rhoA);
    fprintf('Number of starts: %d\n',nStarts);
    fprintf('============================================================\n');

    optimOptions = optimset( ...
        'Display',opts.optimizer_display, ...
        'MaxFunEvals',opts.max_fun_evals, ...
        'MaxIter',opts.max_iter, ...
        'TolX',opts.tol_x, ...
        'TolFun',opts.tol_fun);

    for s = 1:nStarts
        activeStart = s;
        K0 = starts(s,:);

        successCountAtStart = totalSuccessfulEvaluations;
        failureCountAtStart = totalFailedEvaluations;

        z0 = gains_to_unconstrained_log( ...
            K0,opts.gain_lb,opts.gain_ub);

        fprintf('\nStart %d/%d\n',s,nStarts);
        fprintf('Initial gains: Kp=%.8g, Ki=%.8g, Kd=%.8g\n', ...
            K0(1),K0(2),K0(3));

        [zFinal,Jfinal,exitflag,output] = fminsearch( ...
            @objective_in_unconstrained_coordinates,z0,optimOptions);

        Kfinal = unconstrained_to_gains_log( ...
            zFinal,opts.gain_lb,opts.gain_ub);

        [Jcheck,nSuccess,nFail,diagFinal] = ...
            evaluate_gain_vector(Kfinal);

        if isfinite(Jcheck)
            Jfinal = Jcheck;
        end

        Kp_final(s) = Kfinal(1);
        Ki_final(s) = Kfinal(2);
        Kd_final(s) = Kfinal(3);
        Objective(s) = Jfinal;
        TrackingCost(s) = diagFinal.tracking_mean;
        AmplitudeCost(s) = diagFinal.amplitude_mean;
        ASaturationPct(s) = diagFinal.A_saturation_pct_mean;
        ExitFlag(s) = exitflag;
        Iterations(s) = output.iterations;
        FunctionEvaluations(s) = output.funcCount;
        SuccessfulEvaluations(s) = ...
            totalSuccessfulEvaluations-successCountAtStart;
        FailedEvaluations(s) = ...
            totalFailedEvaluations-failureCountAtStart;

        startsTable = table( ...
            startNumber,Kp_start,Ki_start,Kd_start, ...
            Kp_final,Ki_final,Kd_final, ...
            Objective,TrackingCost,AmplitudeCost,ASaturationPct, ...
            ExitFlag,Iterations,FunctionEvaluations, ...
            SuccessfulEvaluations,FailedEvaluations);

        writetable(startsTable,startsCsv);

        checkpoint = struct();
        checkpoint.setpoint = T4_des;
        checkpoint.options = opts;
        checkpoint.starts = startsTable;
        checkpoint.last_start_successful_cases = nSuccess;
        checkpoint.last_start_failed_cases = nFail;
        save(checkpointMat,'checkpoint','-v7');
    end

    validStarts = isfinite(Objective) & ...
        Objective < 0.99*opts.failure_penalty & ...
        SuccessfulEvaluations > 0;

    if ~any(validStarts)
        error(['Fixed-PID optimization failed: no start produced a ' ...
            'valid objective below %.6g.'],opts.failure_penalty);
    end

    validIndices = find(validStarts);
    [bestObjective,localBest] = min(Objective(validStarts));
    bestIndex = validIndices(localBest);

    Kstar = [ ...
        Kp_final(bestIndex), ...
        Ki_final(bestIndex), ...
        Kd_final(bestIndex)];

    TUNE = struct();
    TUNE.setpoint = T4_des;
    TUNE.Kstar = Kstar;
    TUNE.Kp_star = Kstar(1);
    TUNE.Ki_star = Kstar(2);
    TUNE.Kd_star = Kstar(3);
    TUNE.best_objective = bestObjective;
    TUNE.tracking_cost = TrackingCost(bestIndex);
    TUNE.amplitude_cost = AmplitudeCost(bestIndex);
    TUNE.A_saturation_pct = ASaturationPct(bestIndex);
    TUNE.beta_ov = opts.beta_ov;
    TUNE.rhoA = opts.rhoA;
    TUNE.options = opts;
    TUNE.starts = startsTable;
    TUNE.evaluations_file = evaluationCsv;

    save(resultMat,'TUNE','-v7');

    gainTable = table( ...
        T4_des,Kstar(1),Kstar(2),Kstar(3), ...
        bestObjective,TrackingCost(bestIndex), ...
        AmplitudeCost(bestIndex),ASaturationPct(bestIndex), ...
        opts.beta_ov,opts.rhoA, ...
        'VariableNames',{ ...
        'Setpoint','Kp_star','Ki_star','Kd_star', ...
        'TrainingObjective','TrackingCost', ...
        'AmplitudeCost','A_Saturation_pct', ...
        'beta_ov','rhoA'});

    writetable(gainTable,resultCsv);

    fprintf('\n============================================================\n');
    fprintf('Optimized fixed PID for T4_des = %.6g\n',T4_des);
    print_final_result(TUNE,resultMat);
    fprintf('============================================================\n');

    function J = objective_in_unconstrained_coordinates(z)
        K = unconstrained_to_gains_log( ...
            z,opts.gain_lb,opts.gain_ub);

        [J,nSuccessfulCases,nFailedCases,diagEval] = ...
            evaluate_gain_vector(K);

        evalCounter = evalCounter+1;

        if nSuccessfulCases > 0 && ...
                isfinite(J) && J < opts.failure_penalty
            status = "valid";
            totalSuccessfulEvaluations = ...
                totalSuccessfulEvaluations+1;
        else
            status = "failed";
            totalFailedEvaluations = ...
                totalFailedEvaluations+1;
        end

        evaluationRow = table( ...
            evalCounter,activeStart, ...
            K(1),K(2),K(3),J, ...
            diagEval.tracking_mean, ...
            diagEval.amplitude_mean, ...
            diagEval.A_saturation_pct_mean, ...
            nSuccessfulCases,nFailedCases,status, ...
            string(datetime('now', ...
                'Format','yyyy-MM-dd HH:mm:ss')), ...
            'VariableNames',{ ...
            'Evaluation','Start','Kp','Ki','Kd', ...
            'Objective','TrackingCost','AmplitudeCost', ...
            'A_Saturation_pct','SuccessfulCases', ...
            'FailedCases','Status','Timestamp'});

        append_table_row(evaluationRow,evaluationCsv);

        fprintf(['Eval %d, start %d: ' ...
            'K=[%.7g %.7g %.7g], J=%.8g, ' ...
            'Jtrack=%.8g, JA=%.8g, Asat=%.2f%%, ' ...
            'successful=%d, failed=%d\n'], ...
            evalCounter,activeStart,K(1),K(2),K(3),J, ...
            diagEval.tracking_mean,diagEval.amplitude_mean, ...
            diagEval.A_saturation_pct_mean, ...
            nSuccessfulCases,nFailedCases);
    end

    function [J,nSuccessfulCases,nFailedCases,diagMean] = ...
            evaluate_gain_vector(K)

        nCases = numel(opts.training_seeds);
        caseCosts = nan(nCases,1);
        trackingCosts = nan(nCases,1);
        amplitudeCosts = nan(nCases,1);
        saturationPcts = nan(nCases,1);

        firstFailure = [];
        nSuccessfulCases = 0;
        nFailedCases = 0;

        for jj = 1:nCases
            seed = opts.training_seeds(jj);

            fixedOpts = opts.fixed_common;
            fixedOpts.Kp = K(1);
            fixedOpts.Ki = K(2);
            fixedOpts.Kd = K(3);
            fixedOpts.t_end_hr = opts.tune_horizon_hr;
            fixedOpts.meas_sigma = opts.meas_sigma;
            fixedOpts.rng_seed = seed;

            try
                OUT = run_pid_fixed_baseline( ...
                    T4_des,fixedOpts);

                [caseCosts(jj),parts] = tuning_cost( ...
                    OUT,opts.beta_ov,opts.rhoA);

                trackingCosts(jj) = parts.tracking;
                amplitudeCosts(jj) = parts.amplitude;
                saturationPcts(jj) = ...
                    parts.A_saturation_pct;

                nSuccessfulCases = nSuccessfulCases+1;
                clear OUT
            catch ME
                warning(ME.identifier, ...
                    ['Fixed-PID simulation failed for ' ...
                    'K=[%g %g %g], seed=%d: %s'], ...
                    K(1),K(2),K(3),seed,ME.message);

                caseCosts(jj) = opts.failure_penalty;
                nFailedCases = nFailedCases+1;

                if isempty(firstFailure)
                    firstFailure = ME;
                end
            end
        end

        diagMean = struct( ...
            'tracking_mean',NaN, ...
            'amplitude_mean',NaN, ...
            'A_saturation_pct_mean',NaN);

        if nSuccessfulCases == 0
            if opts.abort_if_all_cases_fail && ...
                    ~isempty(firstFailure)
                rethrow(firstFailure);
            end
            J = opts.failure_penalty;
            return;
        end

        J = mean(caseCosts);

        validTrack = isfinite(trackingCosts);
        validAmp = isfinite(amplitudeCosts);
        validSat = isfinite(saturationPcts);

        if any(validTrack)
            diagMean.tracking_mean = ...
                mean(trackingCosts(validTrack));
        end
        if any(validAmp)
            diagMean.amplitude_mean = ...
                mean(amplitudeCosts(validAmp));
        end
        if any(validSat)
            diagMean.A_saturation_pct_mean = ...
                mean(saturationPcts(validSat));
        end

        if ~isfinite(J)
            J = opts.failure_penalty;
        end
    end
end


function opts = apply_defaults(opts,projectRoot,T4_des)
    setpointTag = numeric_file_tag(T4_des);

    opts = set_default(opts,'tune_horizon_hr',80);
    opts = set_default(opts,'training_seeds',1);
    opts = set_default(opts,'meas_sigma',0);

    % Scientifically relevant region identified from the actuator map.
    % Kd uses a tiny positive lower bound to support log coordinates;
    % values near 1e-10 are numerically equivalent to Kd=0 here.
    opts = set_default(opts,'gain_lb', ...
        [1e-5,1e-9,1e-10]);
    opts = set_default(opts,'gain_ub', ...
        [5e-2,2e-4,1e-2]);
    opts = set_default(opts,'initial_guess', ...
        [1.5e-2,1e-5,1e-8]);

    opts = set_default(opts,'n_starts',5);
    opts = set_default(opts,'max_fun_evals',80);
    opts = set_default(opts,'max_iter',60);
    opts = set_default(opts,'tol_x',1e-4);
    opts = set_default(opts,'tol_fun',1e-4);
    opts = set_default(opts,'optimizer_display','iter');

    opts = set_default(opts,'force_retune',false);
    opts = set_default(opts,'failure_penalty',1e12);
    opts = set_default(opts,'abort_if_all_cases_fail',true);
    opts = set_default(opts,'fixed_common',struct());

    % Empty means: read directly from rapid_controller_defaults.m.
    opts = set_default(opts,'beta_ov',[]);
    opts = set_default(opts,'rhoA',[]);

    opts = set_default(opts,'output_dir', ...
        fullfile(projectRoot,'results', ...
        sprintf('fixed_pid_tuning_T4_%s',setpointTag)));
end


function [beta_ov,rhoA] = ...
        resolve_paper_cost_parameters(T4_des,opts)

    beta_ov = opts.beta_ov;
    rhoA = opts.rhoA;

    if ~isempty(beta_ov) && ~isempty(rhoA)
        return;
    end

    require_file('rapid_apply_default_options');
    require_file('rapid_constants');
    require_file('rapid_build_schedule');
    require_file('rapid_controller_defaults');

    scheduleOpts = struct();
    scheduleOpts.dt_seconds = 0.1;
    scheduleOpts.sim_hours = opts.tune_horizon_hr;
    scheduleOpts.do_plots = false;
    scheduleOpts.verbose = false;
    scheduleOpts.do_save_pdf = false;
    scheduleOpts.output_dir = opts.output_dir;
    scheduleOpts.rng_seed = 1;

    scheduleOpts = ...
        rapid_apply_default_options(scheduleOpts);
    CONST = rapid_constants();
    SCHED = rapid_build_schedule(scheduleOpts,CONST);
    CTRL = rapid_controller_defaults(T4_des,SCHED);

    if isempty(beta_ov)
        if ~isfield(CTRL,'beta_ov')
            error('CTRL.beta_ov is not defined.');
        end
        beta_ov = CTRL.beta_ov;
    end

    if isempty(rhoA)
        if ~isfield(CTRL,'rhoA')
            error('CTRL.rhoA is not defined.');
        end
        rhoA = CTRL.rhoA;
    end
end


function validate_options(opts)
    if opts.tune_horizon_hr <= 0
        error('opts.tune_horizon_hr must be positive.');
    end

    if isempty(opts.training_seeds) || ...
            any(~isfinite(opts.training_seeds)) || ...
            any(mod(opts.training_seeds,1) ~= 0)
        error('opts.training_seeds must contain integer seeds.');
    end

    if numel(opts.gain_lb) ~= 3 || ...
            numel(opts.gain_ub) ~= 3
        error('gain_lb and gain_ub must contain [Kp Ki Kd].');
    end

    if any(opts.gain_lb <= 0) || ...
            any(opts.gain_ub <= opts.gain_lb)
        error(['Log-coordinate gain bounds require ' ...
            '0 < lower < upper.']);
    end

    if numel(opts.initial_guess) ~= 3
        error('opts.initial_guess must contain [Kp Ki Kd].');
    end

    if opts.n_starts < 1 || mod(opts.n_starts,1) ~= 0
        error('opts.n_starts must be a positive integer.');
    end

    if ~isscalar(opts.beta_ov) || ...
            ~isfinite(opts.beta_ov) || opts.beta_ov <= 0
        error('beta_ov must be a positive finite scalar.');
    end

    if ~isscalar(opts.rhoA) || ...
            ~isfinite(opts.rhoA) || opts.rhoA < 0
        error('rhoA must be a nonnegative finite scalar.');
    end
end


function starts = make_start_points(opts)
    lb = opts.gain_lb(:)';
    ub = opts.gain_ub(:)';

    candidateStarts = [ ...
        opts.initial_guess(:)'; ...
        5e-3,1e-6,1e-10; ...
        1.0e-2,5e-6,1e-8; ...
        2.0e-2,3e-5,1e-6; ...
        3.0e-2,1e-4,1e-4];

    starts = nan(opts.n_starts,3);
    nPreset = min(opts.n_starts,size(candidateStarts,1));

    for i = 1:nPreset
        starts(i,:) = clip_log_interior( ...
            candidateStarts(i,:),lb,ub);
    end

    rng(20260806,'twister');

    for i = (nPreset+1):opts.n_starts
        q = rand(1,3);
        starts(i,:) = 10.^( ...
            log10(lb)+q.*(log10(ub)-log10(lb)));
    end
end


function K = clip_log_interior(K,lb,ub)
    K = K(:)';
    tiny = 1e-8;
    logLb = log10(lb);
    logUb = log10(ub);
    logK = log10(max(K,lb));

    logK = min(logUb-tiny*(logUb-logLb), ...
        max(logLb+tiny*(logUb-logLb),logK));
    K = 10.^logK;
end


function z = gains_to_unconstrained_log(K,lb,ub)
    K = clip_log_interior(K,lb,ub);

    logLb = log10(lb);
    logUb = log10(ub);
    q = (log10(K)-logLb)./(logUb-logLb);
    q = min(1-1e-12,max(1e-12,q));

    z = log(q./(1-q));
end


function K = unconstrained_to_gains_log(z,lb,ub)
    z = max(-40,min(40,z));
    q = 1./(1+exp(-z));

    logK = log10(lb)+q.*(log10(ub)-log10(lb));
    K = 10.^logK;
end


function [J,parts] = tuning_cost(OUT,beta_ov,rhoA)
% Full-horizon counterpart of the paper's one-window APID/RAPID cost.

    if ~isfield(OUT,'time_hr') || ...
            ~isfield(OUT,'T4_true')
        error('OUT must contain time_hr and T4_true.');
    end

    tMin = 60*OUT.time_hr(:);
    y = OUT.T4_true(:);
    ref = extract_setpoint(OUT);

    n = min(numel(tMin),numel(y));
    tMin = tMin(1:n);
    y = y(1:n);

    valid = isfinite(tMin) & isfinite(y);
    tMin = tMin(valid);
    y = y(valid);

    if numel(tMin) < 2 || ~isfinite(ref)
        error('Insufficient finite tracking data.');
    end

    % Same sign convention as rapid_window_cost_ISE_one_robust.m:
    % positive error is overshoot above the reference.
    err = y-ref;
    pos = max(err,0);
    neg = min(err,0);

    Jtracking = trapz( ...
        tMin,beta_ov*pos.^2+neg.^2);

    if ~isfield(OUT,'A_cmd')
        error('OUT must contain A_cmd.');
    end

    A = OUT.A_cmd(:);
    A = A(isfinite(A));

    if isempty(A)
        Jamplitude = 0;
        satPct = NaN;
    else
        % The paper adds rhoA*A_next^2 once per control window.
        Jamplitude = rhoA*sum(A.^2);

        Amax = extract_Amax(OUT);
        if isfinite(Amax) && Amax > 0
            tol = max(1e-10,1e-8*Amax);
            satPct = 100*mean(abs(A-Amax) <= tol);
        else
            satPct = NaN;
        end
    end

    J = Jtracking+Jamplitude;

    if ~isfinite(J)
        error('Nonfinite tuning objective.');
    end

    parts = struct();
    parts.tracking = Jtracking;
    parts.amplitude = Jamplitude;
    parts.total = J;
    parts.A_saturation_pct = satPct;
end


function ref = extract_setpoint(OUT)
    if isfield(OUT,'setpoint')
        ref = OUT.setpoint;
    elseif isfield(OUT,'T4_des')
        ref = OUT.T4_des;
    elseif isfield(OUT,'P') && ...
            isstruct(OUT.P) && ...
            isfield(OUT.P,'T4_des')
        ref = OUT.P.T4_des;
    else
        error('Could not find the setpoint in OUT.');
    end
end


function Amax = extract_Amax(OUT)
    if isfield(OUT,'P') && ...
            isstruct(OUT.P) && ...
            isfield(OUT.P,'A_max')
        Amax = OUT.P.A_max;
    elseif isfield(OUT,'A_max')
        Amax = OUT.A_max;
    else
        Amax = NaN;
    end
end


function projectRoot = locate_project_root(startDir)
    projectRoot = startDir;
    current = startDir;

    for k = 1:5
        if exist(fullfile(current,'common'),'dir') || ...
                exist(fullfile(current,'apid'),'dir') || ...
                exist(fullfile(current,'src'),'dir')
            projectRoot = current;
            return;
        end

        parent = fileparts(current);
        if strcmp(parent,current)
            break;
        end
        current = parent;
    end
end


function add_project_paths(projectRoot,thisDir)
    folders = { ...
        projectRoot, ...
        thisDir, ...
        fullfile(projectRoot,'src'), ...
        fullfile(projectRoot,'common'), ...
        fullfile(projectRoot,'common','plant'), ...
        fullfile(projectRoot,'common','timing'), ...
        fullfile(projectRoot,'common','utils'), ...
        fullfile(projectRoot,'apid'), ...
        fullfile(projectRoot,'apid','src'), ...
        fullfile(projectRoot,'rapid'), ...
        fullfile(projectRoot,'rapid','src'), ...
        fullfile(projectRoot,'experiments')};

    for i = 1:numel(folders)
        if exist(folders{i},'dir')
            addpath(folders{i});
        end
    end
end


function require_file(functionName)
    if exist(functionName,'file') ~= 2
        error('Required MATLAB function was not found: %s.m', ...
            functionName);
    end
end


function opts = set_default(opts,name,value)
    if ~isfield(opts,name) || isempty(opts.(name))
        opts.(name) = value;
    end
end


function archive_existing_file(filename)
    if ~exist(filename,'file')
        return;
    end

    [folder,name,ext] = fileparts(filename);
    stamp = char(datetime('now', ...
        'Format','yyyyMMdd_HHmmss'));

    archived = fullfile(folder, ...
        sprintf('%s_previous_%s%s',name,stamp,ext));
    movefile(filename,archived);
end


function append_table_row(rowTable,filename)
    if exist(filename,'file')
        writetable(rowTable,filename, ...
            'WriteMode','append', ...
            'WriteVariableNames',false);
    else
        writetable(rowTable,filename);
    end
end


function tag = numeric_file_tag(value)
    tag = sprintf('%.6g',value);
    tag = strrep(tag,'-','m');
    tag = strrep(tag,'.','p');
end


function print_final_result(TUNE,resultMat)
    fprintf('Kp* = %.12g\n',TUNE.Kstar(1));
    fprintf('Ki* = %.12g\n',TUNE.Kstar(2));
    fprintf('Kd* = %.12g\n',TUNE.Kstar(3));
    fprintf('Objective = %.12g\n',TUNE.best_objective);

    if isfield(TUNE,'tracking_cost')
        fprintf('Tracking cost = %.12g\n',TUNE.tracking_cost);
    end
    if isfield(TUNE,'amplitude_cost')
        fprintf('Amplitude cost = %.12g\n',TUNE.amplitude_cost);
    end
    if isfield(TUNE,'A_saturation_pct')
        fprintf('Amplitude saturation = %.3f%%\n', ...
            TUNE.A_saturation_pct);
    end
    if isfield(TUNE,'beta_ov')
        fprintf('beta_ov = %.12g\n',TUNE.beta_ov);
    end
    if isfield(TUNE,'rhoA')
        fprintf('rhoA = %.12g\n',TUNE.rhoA);
    end

    fprintf('Saved to:\n%s\n',resultMat);
end
