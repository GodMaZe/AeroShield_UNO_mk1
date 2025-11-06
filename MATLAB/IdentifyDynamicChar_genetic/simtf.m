% funkcia na simulaciu regulacie SISO s neuro-regulatorom, bez simulinku
function[x]=simtf(Gs,u,dt,x)
% u = Control input
% Gs = The model to model
% x0 = initial state (pass this parameter when you want to reset the state
% to initial)
nxs = numel(x);
nxsm = nxs - 1;
B = Gs.Numerator(end:-1:1);
A = Gs.Denominator(end:-1:1);

fx = @(A, B, u, x) (B * u' - A(1:end - 1) * x')/A(end);

K = zeros(nxs, 4);

K(1:nxsm, 1) = x(2:nxs) * dt;
K(nxs, 1) = fx(A, B, u, x) * dt;

for i=1:2
    for k=1:nxsm
        K(k, i + 1) = (x(k + 1) + K(k + 1, i)/2) * dt;
    end
    K(nxs, i + 1) = fx(A, B, u, x + K(:, i)'/2) * dt;
end

K(1:nxsm, end) = (x(2:nxs) + K(2:nxs, 3)) * dt;
K(nxs, 4) = fx(A, B, u, x + K(:, 3)') * dt;

x = x + (1/6*K*[1 2 2 1]')';
end