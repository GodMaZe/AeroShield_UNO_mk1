function [f,h,Fx,Hx] = create_nonlinear_pendulum(pendulum, Ts, use_saturation)
arguments (Input)
    pendulum = Pendulum();
    Ts = 0.05;
    use_saturation = false;
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

Fc = @(x) pendulum.mu*pendulum.g*(pendulum.m1 + pendulum.m2)*NL(x(1));

f_cont = @(x, u) [x(2); -1/pendulum.I_T * (pendulum.xi*x(2) + Fc(x) + pendulum.G_1*sin(x(1)))] ...
    + [0; 1/pendulum.I_T] * u(1);

f = @(x,u) rk4_step(f_cont, x, u, Ts); % Important to use the rk4 step for precise and safe integration
h = @(x,u) x(1);

Fx = @(x, u) discrete_jacobian(f_cont, x, u, Ts);
Hx = @(x, u) [1, 0];

end

