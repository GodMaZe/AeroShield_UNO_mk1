function y = sticky_friction(vel, tau_coloumb, tau_brk, vel_S)
%STICKY FRICTION Mathematically modeled sticky friction.
arguments (Input)
    vel;
    tau_coloumb;
    tau_brk = 5.05;
    vel_S = 0.1*sqrt(2);
end
arguments (Output)
    y;
end
    v = (vel./vel_S);
    y = -sqrt(2*exp(1)) * (tau_brk - abs(tau_coloumb)) .* (exp(- v.^2) .* v);
end