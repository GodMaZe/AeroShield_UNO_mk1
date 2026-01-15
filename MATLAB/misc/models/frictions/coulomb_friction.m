function y = coulomb_friction(vel, tau_c, hc, vel_dry)
%VISCOUS FRICTION Mathematically modeled coloumb friction.
arguments (Input)
    vel;
    tau_c = 3.7;
    hc = 1e5;
    vel_dry = 0.01;
end
arguments (Output)
    y;
end

    y = -tau_c * tanh(hc/vel_dry * vel);
end