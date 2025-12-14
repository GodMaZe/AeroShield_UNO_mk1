function [t, xs] = rk4(f, x, u, Tstop, dt)
%RK4 Numerically solve a differential equation using RK4 algorithm.
%
% Example of usage:
% f = @(x, u) [x(2); -1/I_T * (xi*x(2) + mu*g*(m1 + m2)*saturation(x(1)) + G_1*sin(x(1)))] + [0; 1/I_T]*u;
% x0 = [pi/4; 0];
% [t, x] = rk4(f_cont, x0, 0, 30);
% plot(t, x); legend('x1','x2'); grid minor, grid on; xlabel('t [s]'); ylabel('states [-]'); title("RK4 damped physical pendulum with input signal");
arguments (Input)
    f;
    x;
    u=0;
    Tstop=10;
    dt=0.01;
end

arguments (Output)
    t;
    xs;
end


nsteps = floor(Tstop/dt) + 1;
t = zeros(1, nsteps);
xs = zeros(size(x, 1), nsteps);
xs(:, 1) = x;

for i=2:nsteps
    xs(:, i) = rk4_step(f, xs(:, i-1), u, dt);
    t(i) = t(i-1) + dt;
end

end