function xsat = saturate(x, min_, max_)
%SATURATE Sature the x parameter within the min and max interval included.
arguments (Input)
    x
    min_ = -1;
    max_ = 1;
end
arguments (Output)
    xsat
end

    if min_ > max_
        error('Minimum value must be less than or equal to maximum value.');
    end

    xsat = max(min_, min(max_, x));
end