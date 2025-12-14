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


k1 = f(x, u);
k2 = f(x + dt_2*k1, u);
k3 = f(x + dt_2*k2, u);
k4 = f(x + dt*k3, u);
x = x + (dt/6) * (k1 + 2*k2 + 2*k3 + k4);
end