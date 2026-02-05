function [f, b, h, Fx, Bu, Hx] = create_nonlinear_pendulum(pendulum, Ts, w_disturbance, w_propeller)
arguments (Input)
    pendulum = Pendulum();
    Ts = 0.05;
    w_disturbance = false;
    w_propeller = false;
end
arguments (Output)
    % Initialize state variables and system matrices
    f;
    b;
    h;
    Fx;
    Bu;
    Hx;
end

dt = Ts;


function x2 = dx2(t, x, u, vel_tol)
    if nargin < 4
        vel_tol = 0.001;
    end
    
    function tau_f = frictions(t, x, u, vel_tol)
        tau_v = viscous_friction(x(2), pendulum.xi);
        tau_c = coulomb_friction(x(2), pendulum.mu*pendulum.g*(pendulum.m1 + pendulum.m2), pendulum.hc, pendulum.omega_dry);
        if abs(x(2)) <= vel_tol && abs(x(1)) <= vel_tol && abs(tau_g) > abs(tau_c)
            tau_c = tau_g;
        end
        tau_s = sticky_friction(x(2), tau_c, pendulum.tau_brk, pendulum.omega_S);
        tau_a = drag_friction(x(2), pendulum.ka, pendulum.kr);
        tau_f = tau_v + tau_c + tau_s + tau_a;
    end

    % The passive effect of the frictional forces is incorrectly modeled
    tau_g = pendulum.G_1*sin(x(1));
    Fs = frictions(t, x, u, vel_tol);
    x2 = (1/pendulum.I_T) * (Fs - tau_g);
    if w_propeller
        x2 = x2 + x(3);
    end
end

function f = Buf(t, x, u)
    f = [0; 0];
    if w_propeller
        f = [0; f];
    end

    if w_disturbance
        f = [f; 0];
    end
end

Bu = @(t, x, u) Buf(t, x, u);
b = @(t, x, u) Bu(t, x, u)*u(1);



if w_propeller
    f_cont = @(t, x, u) [
        x(2);
        dx2(t, x, u);
        -5*x(3)+15*u(1); %(pendulum.I_T/pendulum.Ku)
        ] + b(t, x, u);
    Hx = @(t, x, u) [1 0 0];
else
    f_cont = @(t, x, u) [
    x(2);
    dx2(t, x, u)
    ] + b(t, x, u);
    Hx = @(t, x, u) [1 0];
end

h = @(t, x, u) x(1);


if w_disturbance
    if w_propeller
        f_cont = @(t, x, u) [x(2); dx2(t, x, x(3)); 0.1; x(4)] + b(t, x, u);
        h = @(t, x, u) x(1) + x(4);
        Hx = @(t, x, u) [1 0 0 1];
    else
        f_cont = @(t, x, u) [x(2); dx2(t, x, u); x(3)] + b(t, x, u);
        h = @(t, x, u) x(1) + x(3);
        Hx = @(t, x, u) [1 0 1];
    end
end


% f = @(t,x,u) euler_step(f_cont, t, x, u, dt);
f = @(t,x,u) rk4_step(f_cont, t, x, u, dt); % Important to use the rk4 step for precise and safe integration

Fx = @(t, x, u) discrete_jacobian(f, t, x, u, dt);

end