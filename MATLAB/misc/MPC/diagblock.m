function D = diagblock(M, p)
    % m = size(M,1);
    % n = size(M,2);
    % D = zeros(m*p, n*p);
    % for i=1:p
    %     D((1+(i-1)*m):(i*m), (1+(i-1)*n):(i*n)) = M;
    % end
    D = kron(eye(p), M);
end