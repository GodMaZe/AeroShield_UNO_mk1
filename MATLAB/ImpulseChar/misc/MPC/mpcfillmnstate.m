function [M, N] = mpcfillmnstate(A, B, p)
    n = size(A,1);
    r = size(B,2);

    M = zeros(n*p, n);
    N = zeros(n*p, r*p);
    
    M(1:n, 1:n) = A;
    N(1:n, 1:r) = B;

    for i=2:p
        M(1+((i-1)*n):(i*n), 1:n) = M(1+((i-2)*n):((i-1)*n), 1:n) * A;
        N(1+((i-1)*n):(i*n), 1:r) = A^(i-1)*B;
    end

    for i=1:p-1
        N((i*n+1):end, (1 + (i*r)):((i+1)*r)) = N(1:(n*p-(i*n)), 1:r);
    end
end