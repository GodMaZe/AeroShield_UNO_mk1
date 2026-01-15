function xdead = deadzone(x, lower_bound, upper_bound)
%DEADZONE Deadzone function that returns zero if the input variable x is
%within the upper and lower bound; included.
arguments (Input)
    x;
    lower_bound = -1;
    upper_bound = 1;
end

arguments (Output)
    xdead;
end
if lower_bound > upper_bound
    error('Lower bound must be less than or equal to upper bound.');
end

xdead = (x < lower_bound | x > upper_bound) .* x;

end