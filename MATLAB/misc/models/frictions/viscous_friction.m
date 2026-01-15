function y = viscous_friction(vel, kv)
%VISCOUS FRICTION Mathematically modeled viscous friction.
arguments (Input)
    vel;
    kv = 1e-6;
end

arguments (Output)
    y;
end

    y = -kv * vel;
end