function [Q, R] = QR_matrix(n, m, w_propeller)
%QR_MATRIX Return the 
arguments (Input)
    n = 2;
    m = 1;
    w_propeller = false;
end

arguments (Output)
    Q;
    R;
end

R = deg2rad(0.015)^2; % Measurement noise (from datasheet)
if n == 3
    if w_propeller
        Q = diag([deg2rad(0.001)^2 deg2rad(0.1)^2 0.001^2]);
    else
        Q = diag([deg2rad(0.001)^2 deg2rad(0.1)^2 0.1^2]);
    end
elseif n == 4
        Q = diag([deg2rad(0.001)^2 deg2rad(0.1)^2 0.001^2 0.1^2]);
else
    Q = diag([deg2rad(0.001)^2 deg2rad(0.1)^2]);
end

end