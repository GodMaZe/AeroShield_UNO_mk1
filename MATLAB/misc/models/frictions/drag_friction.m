function y = drag_friction(vel, ka, kr)
%DRAG FRICTION Mathematically modeled drag friction, caused by air resistance.
arguments (Input)
    vel;
    ka = 0.1;
    kr = 0.01;
end
arguments (Output)
    y;
end
    y = - ka * vel.^2 .* tanh(kr * vel);
end