function [f,h,Fx,Hx] = create_nonlinear_pendulum(pendulum, Ts, use_saturation, w_disturbance)
arguments (Input)
    pendulum = Pendulum();
    Ts = 0.05;
    use_saturation = false;
    w_disturbance = false;
end
arguments (Output)
    % Initialize state variables and system matrices
    f
    h
    Fx
    Hx
end


if use_saturation
    NL = @(x) min(1, max(-1, x));
else
    NL = @(x) tanh(x);
end

dt = Ts;

Fc = @(x) pendulum.mu*pendulum.g*(pendulum.m1 + pendulum.m2)*NL(x(2));
Fs = @(x) pendulum.xi*x(2) + Fc(x);

f_cont = @(x, u) [x(1) + x(2) * dt; (dt/pendulum.I_T * ((Fs(x) + pendulum.G_1*sin(x(1))) - u(1)))];
h = @(x,u) sin(x(1));
Hx = @(x, u) [cos(x(1)) 0];

if w_disturbance
    f_cont = @(x, u) [x(1) + x(2) * dt; (dt/pendulum.I_T * ((Fs(x) + pendulum.G_1*sin(x(1))) - u(1))); x(3)];

    h = @(x, u) sin(x(1) + x(3));

    Hx = @(x, u) [cos(x(1)) 0 cos(x(3))];
end

f = f_cont; % @(x,u) rk4_step(f_cont, x, u, dt); % Important to use the rk4 step for precise and safe integration
Fx = @(x, u) discrete_jacobian(f_cont, x, u, dt);


end