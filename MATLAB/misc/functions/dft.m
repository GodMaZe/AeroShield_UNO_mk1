function [X] = dft(x)
%DFT Compute the 1-D discrete Fourier Transform of the provided signal x.
arguments (Input)
    x;
end

arguments (Output)
    X;
end

% sum_{0}^{N-1} x_n * e^{-i*2*pi*k*n/N}
% n - sample number
% k - sample frequency

N = numel(x) - 1;
n = 0:N;
k = n';
e = exp(-1j*2*pi*k*n/N);
X = e * x.';
end