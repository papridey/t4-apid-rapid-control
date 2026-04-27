function dydt_min = rapid_ode_perturbed_model(t_min, y, EF_time_min, pert, P)
    EF_base = EF_time_min(t_min);

    gammaext_T4 = P.gammaext_T4 * (1 + pert.gext_scale);
    M_fun       = P.M_fun       * (1 + pert.M_scale);
    alpha1      = P.alpha1      * (1 + pert.alpha1_scale);

    x      = y(1:4);
    u_f    = y(5);
    u_hist = y(6:15);
    nkx21  = y(16);

    alpha  = P.alpha  * (1 + pert.alpha_scale);
    K      = P.K      * (1 + pert.K_scale);
    gamma1 = P.gamma1 * (1 + pert.gamma1_scale);

    Ndelay = P.N;
    a      = P.a;
    U = zeros(Ndelay,1);
    U(1) = u_f;

    du_f    = alpha1 * EF_base^P.n / (K^P.n + EF_base^P.n) - gamma1 * u_f;
    du_hist = (-a * eye(Ndelay) + a * diag(ones(1, Ndelay-1), -1)) * u_hist + a * U;
    dNKX21  = alpha * u_hist(end) - P.gamma_NKX21 * nkx21 + P.alpha_NKX21_offset;

    kappa = P.kappa * (1 + pert.kappa_scale);
    K2    = P.K2    * (1 + pert.K2_scale);
    k_1   = kappa * ((nkx21 / K2)^3 / (1 + (nkx21 / K2)^3));

    gamma_I     = P.gamma_I     * (1 + pert.gammaI_scale);
    gamma_Tg    = P.gamma_Tg    * (1 + pert.gammaTg_scale);
    gammaint_T4 = P.gammaint_T4 * (1 + pert.gint_scale);

    dx1 = -k_1 * x(3) * x(1) - gamma_I * x(1) + P.alpha_I;
    dx2 =  k_1 * x(3) * x(1) - M_fun * x(2) - gammaint_T4 * x(2);
    dx3 =  P.alpha_TG - k_1 * x(3) * x(1) - gamma_Tg * x(3);

    d4  = pert.d4_bias + pert.d4_amp * sin(2*pi*(t_min / pert.d4_period_min + pert.d4_phase));
    dx4 =  M_fun * x(2) - gammaext_T4 * x(4) - d4;

    dydt_sec = [dx1; dx2; dx3; dx4; du_f; du_hist; dNKX21];
    dydt_min = 60 * dydt_sec;
end
