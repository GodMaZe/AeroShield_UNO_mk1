function [X] = rk4_step(f, x, u, dt)
%RK4_STEP Summary of this function goes here
%   Detailed explanation goes here
arguments (Input)
    f;
    x;
    u=0;
    dt=0.01;
end
arguments (Output)
    X
end
k1 = dt * f(x, u);
k2 = dt * f(x + 0.5*k1, u);
k3 = dt * f(x + 0.5*k2, u);
k4 = dt * f(x + k3, u);
K = (1/6) * (k1 + 2*k2 + 2*k3 + k4);
X = x + K;
end