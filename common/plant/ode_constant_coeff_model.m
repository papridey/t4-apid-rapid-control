function dydt_min = ode_constant_coeff_model(t_min, y, EF_time_min, P)
%ODE_CONSTANT_COEFF_MODEL Nominal (constant-coefficient) EF->T4 model.
%
% State ordering matches the APID/RAPID scripts:
%   y(1:4)   = x1,x2,x3,x4
%   y(5)     = u_f
%   y(6:15)  = u_hist (N=10 delay chain)
%   y(16)    = nkx21
%
% Units:
%   t_min is minutes. Internally we compute dynamics in seconds and scale.

    EF = EF_time_min(t_min);

    x      = y(1:4);
    u_f    = y(5);
    u_hist = y(6:15);
    nkx21  = y(16);

    % EF -> u_f
    du_f = P.alpha1 * EF^P.n / (P.K^P.n + EF^P.n) - P.gamma1 * u_f;

    % delay chain
    Ndelay = P.N;
    a      = P.a;
    U = zeros(Ndelay,1);
    U(1) = u_f;
    du_hist = (-a * eye(Ndelay) + a * diag(ones(1, Ndelay-1), -1)) * u_hist + a * U;

    % nkx21
    dNKX21  = P.alpha * u_hist(end) - P.gamma_NKX21 * nkx21 + P.alpha_NKX21_offset;

    % k1(nkx21)
    k_1 = P.kappa * ((nkx21 / P.K2)^3 / (1 + (nkx21 / P.K2)^3));

    % immune/thyroid blocks
    dx1 = -k_1 * x(3) * x(1) - P.gamma_I * x(1) + P.alpha_I;
    dx2 =  k_1 * x(3) * x(1) - P.M_fun * x(2) - P.gammaint_T4 * x(2);
    dx3 =  P.alpha_TG - k_1 * x(3) * x(1) - P.gamma_Tg * x(3);
    dx4 =  P.M_fun * x(2) - P.gammaext_T4 * x(4);

    dydt_sec = [dx1; dx2; dx3; dx4; du_f; du_hist; dNKX21];
    dydt_min = 60 * dydt_sec;
end
