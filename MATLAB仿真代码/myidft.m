function [nn, xn] = myidft(Xk, fs, N)
    % DFT
    n = 0:1:N-1;
    k = n;
    WN= exp(-1j*2*pi/N);
    nk = n' * k;            % N^2 times multiply
    xn = (Xk(1:N) * WN.^(-nk))/N;  % N^3 times multiply
    nn = 0:1/fs:1/fs*(N-1);
end