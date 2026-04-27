function P = rapid_get_nominal_plant_params()
    P.alpha1      = 3.308;
    P.gammaext_T4 = 0.01;
    P.M_fun       = 0.15;

    P.alpha  = 20 * 8.43812;
    P.K      = 10 * 0.1195;
    P.gamma1 = 0.0296;
    P.n      = 2;

    P.N = 10;
    P.a = 0.0001 * P.N;

    P.alpha_NKX21_offset = 0.25;
    P.gamma_NKX21        = 0.01;

    P.kappa = 1;
    P.K2    = 1000;

    P.alpha_TG    = 1;
    P.alpha_I     = 1;
    P.gamma_I     = 0.004;
    P.gamma_Tg    = 0.04;
    P.gammaint_T4 = 0.1504;
end
