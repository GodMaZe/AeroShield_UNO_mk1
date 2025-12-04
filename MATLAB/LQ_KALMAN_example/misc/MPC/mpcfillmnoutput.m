function [M, N] = mpcfillmnoutput(A, B, C, p)
    n = size(A, 1);
    r = size(B, 2);
    m = size(C, 1);

    M = zeros(m*p, n);
    N = zeros(m*p, r*p);

    % Implementation of filling the matrix M = [C*A, C*A^(2), ..., C*A^(p)]'

    M(1, :) = C * A;
    N(1, 1) = C * B;


    for i=2:p
        M(i, :) = M(i-1, :) * A;
        N(i, 1) = C * A^(i-1) * B;
    end

    for i=1:p-1
        N((i+1):end, i+1) = N(1:(p-i), 1);
    end
end