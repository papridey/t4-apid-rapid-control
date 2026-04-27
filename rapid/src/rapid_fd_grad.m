function g = rapid_fd_grad(fun, x, epsx)
    Jp = fun(x + epsx);
    Jm = fun(x - epsx);
    g = (Jp - Jm) / (2 * epsx);
end
