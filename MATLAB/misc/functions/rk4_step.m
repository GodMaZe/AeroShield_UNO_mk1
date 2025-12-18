function [x] = rk4_step(f, x, u, dt)
%RK4_STEP Summary of this function goes here
%   Detailed explanation goes here
arguments (Input)
    f;
    x;
    u=0;
    dt=0.01;
end
arguments (Output)
    x
end

dt_2 = dt/2;
% xorg = x;

% nxs = numel(x);
% nxsm = nxs - 1;
% 
% K = zeros(nxs, 4);
% 
% K(1:nxsm, 1) = x(2:nxs) * dt;
% K(nxs, 1) = [0 1] * f(x, u) * dt;
% 
% for i=1:2
%     for k=1:nxsm
%         K(k, i + 1) = (x(k + 1) + K(k + 1, i)/2 * dt);
%     end
%     K(nxs, i + 1) = [0 1] * f(x + K(:, i)'/2, u)  * dt;
% end
% 
% K(1:nxsm, end) = (x(2:nxs) + K(2:nxs, 3)) * dt;
% K(nxs, 4) = [0 1] * f(x + K(:, 3)' * dt, u);
% 
% x = x + (dt/6*K*[1 2 2 1]');

k1 = f(x, u);
k2 = f(x + dt_2*k1, u);
k3 = f(x + dt_2*k2, u);
k4 = f(x + dt*k3, u);
% x(2) = (dt/6) * (k1(2) + 2*k2(2) + 2*k3(2) + k4(2));
% x(1) = xorg(1) + x(2) * dt;
% x = (dt/6) * (k1 + 2*k2 + 2*k3 + k4);
% x(1) = xorg(1) + x(2) * dt;
x = x + (dt/6) * (k1(2) + 2*k2(2) + 2*k3(2) + k4(2));
end