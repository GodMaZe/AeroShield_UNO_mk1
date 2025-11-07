function [fit] = fitness_pid(pop, Gs, r, t)
N = size(pop, 1);
fit = zeros(N,1);
npoles = numel(Gs.den) - 1;

c1 = 100; % weight for error
c2 = 25; % weight for error derivative
c3 = 10;

umax = 100;
umin = -100;
dt = mean(diff(t));

parfor i = 1:N
    chrom = pop(i, :);
    if length(chrom) ~= 3
        error('Chromosome length mismatch: expected 3, got %d', length(chrom));
    end

    P = chrom(1);
    I = chrom(2);
    D = chrom(3);
    
    x0 = zeros(npoles, 1);

    [tt,y,dy,w,e,de,u,usat] = sim_pid_VIR25(P, I, D, Gs, r, t, x0);

    IAE = sum(abs(e)) * dt; % Integral of Absolute Error
    IADE = sum(abs(de)) * dt; % Integral of Absolute Derivative of Error
    IUOUTBOUNDS = sum(u < umin | u > umax); % Integral of Control input out of the bounds set by the real system

    fit(i) = c1 * IAE + c2 * IADE + c3 * IUOUTBOUNDS;
    % fprintf("FIT = %8.3f | IAE = %8.3f | IADE = %8.3f | IUOUT = %8.3f\n", fit(i), c1 * IAE, c2 * IADE, c3 * IUOUTBOUNDS)
    if isnan(fit(i))
        fit(i) = Inf; % Assign a large fitness value if NaN
    end
end

end