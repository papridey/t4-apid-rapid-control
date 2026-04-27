function g = fd_grad(fun, x, epsx)
%FD_GRAD Central finite-difference gradient for a scalar function.

    Jp = fun(x + epsx);
    Jm = fun(x - epsx);
    g = (Jp - Jm) / (2 * epsx);
end
