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
    NL = @(x) tanh(x*1e6);
end

dt = Ts;


function x2 = dx2(t, x, u, vel_tol)
    if nargin < 4
        vel_tol = 1e-4;
    end

    iscoloumb = false;
    
    function tau_c = friction_coloumb(t, x, u, vel_tol)
        
        if abs(x(2)) <= vel_tol
            if t == 0 && x(1) ~= 0
                tau_c = pendulum.mu*pendulum.g*(pendulum.m1 + pendulum.m2)*NL(x(2));
            else
                tau_c = -pendulum.G_1*sin(x(1));
            end
            
            fprintf("Inside coloumb!\n");
            iscoloumb = true;
        else
            tau_c = pendulum.mu*pendulum.g*(pendulum.m1 + pendulum.m2)*NL(x(2));
        end
    end
    % The passive effect of the frictional forces is incorrectly modeled
    Fs = pendulum.xi*x(2) + friction_coloumb(t, x, u, vel_tol);
    x2 = (1/pendulum.I_T) * (-Fs - pendulum.G_1*sin(x(1)) + u(1));
    if iscoloumb
        fprintf("x2 = %f\n", x2);
    end
end

f_cont = @(t, x, u) [
    x(2);
    dx2(t, x, u)
    ];
h = @(t, x,u) x(1);
Hx = @(t, x, u) [1 0];

if w_disturbance
    f_cont = @(x, u) [x(2); dx2(t, x, u); x(3)];

    h = @(x, u) x(1) + x(3);

    Hx = @(x, u) [1 0 1];
end


% f = @(t,x,u) euler_step(f_cont, t, x, u, dt);
f = @(t,x,u) rk4_step(f_cont, t, x, u, dt); % Important to use the rk4 step for precise and safe integration
% f = f_cont;

Fx = @(t, x, u) discrete_jacobian(f, t, x, u, dt);

end