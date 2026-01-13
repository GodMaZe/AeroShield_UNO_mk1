function [f] = create_nonlinear_pendulum_identify(pendulum, x, Ts)
arguments (Input)
    x;
    Ts = 0.05;
    pendulum = Pendulum()
end
arguments (Output)
    % Initialize state variables and system matrices
    f;
end

% x - parameters
% x(1) = mu
% x(2) = Ksign
% x(3) = velTol
% x(4) = xi
% x(5) = Ku

mu, Ksign, velTol, xi, Ku = x;

NL = @(x) tanh(Ksign * x);

function x2 = dx2(t, x, u, vel_tol)
    if nargin < 4
        vel_tol = 0.001;
    end

    function tau_c = friction_coloumb(t, x, u, vel_tol)
        
        tau_c = mu*pendulum.g*(pendulum.m1 + pendulum.m2)*NL(dy);
        if abs(dy) <= vel_tol && abs(y) <= vel_tol && abs(tau_g) > abs(tau_c)
            tau_c = -tau_g;
        end
    end

    % The passive effect of the frictional forces is incorrectly modeled
    tau_g = G_1*sin(y);
    Fs = xi*dy + friction_coloumb(t, x, u, vel_tol);
    x2 = (1/I_T) * (-Fs - tau_g);
end

b = @(t, x, u) [0; Ku/I_T*u(1)];

f_cont = @(t, x, u) [
    x(2);
    dx2(t, x, u, velTol)
    ] + b(t, x, u);

f = @(t,x,u) rk4_step(f_cont, t, x, u, Ts); % Important to use the rk4 step for precise and safe integration

end