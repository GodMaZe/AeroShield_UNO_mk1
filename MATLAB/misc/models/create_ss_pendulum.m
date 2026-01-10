function [A, B, C, D] = create_ss_pendulum(pendulum, Ts)
%create_ss_pendulum function is used when creating a continues state-space
%model representation of the AeroShield pendulum system.
arguments (Input)
    pendulum = Pendulum();
    Ts = -1;
end
arguments (Output)
    A;
    B;
    C;
    D;
end

A = [0 1;
    % chyba v implementacii sucheho trenia: Fc =
    %  * tanh(x2) <- ako
    % tanh implementovat
    % 
    -pendulum.G_1/pendulum.I_T -(pendulum.xi+pendulum.mu*pendulum.g*(pendulum.m1 + pendulum.m2))/pendulum.I_T];
B = [0; pendulum.Ku/pendulum.I_T];
C = [1 0];
D = 0;

if Ts > 0
    [A, B, C, D] = ssdata(c2d(ss(A,B,C,D), Ts));
end

end