function [x] = euler_step(f, t, x, u, dt)
%RK4_STEP Summary of this function goes here
%   Detailed explanation goes here
arguments (Input)
    f;
    t;
    x;
    u=0;
    dt=0.01;
end
arguments (Output)
    x
end
N = size(x, 1);

for i=N:-1:1
    dx = f(t, x, u);
    x(i) = x(i) + dt * dx(i);
end
end