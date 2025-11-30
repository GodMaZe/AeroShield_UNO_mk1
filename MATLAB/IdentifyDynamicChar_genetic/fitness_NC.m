function [fit] = fitness_NC(pop, layers, Gs, ref, t, noise, noise_amp,A_tilde,B_tilde,C_tilde,Q,R,P,x_hat)
N = size(pop, 1);
fit = zeros(N,1);
npoles = numel(Gs.den) - 1;

c1 = 10; % weight for error (e)
c2 = 1; % weight for control effort (de)
c3 = 0.45; % max(u)
c4 = 0.001; % Change in the control input in time (du)

umax = 100;
umin = 0;
dt = mean(diff(t));

n_input = layers(1);
n_h1    = layers(2);
n_h2    = layers(3);

lenW1 = n_h1 * n_input;   % W1: n_h1 x n_input
lenW2 = n_h2 * n_h1;      % W2: n_h2 x n_h1
lenW3 = 1     * n_h2;     % W3: 1    x n_h2
expectedLen = lenW1 + lenW2 + lenW3;

is_noise = false;
noise_K = 1;

if nargin > 5
    is_noise = noise;
end
if nargin > 6
    noise_K = noise_amp;
end


parfor i = 1:N
    chrom = pop(i, :);
    if length(chrom) ~= expectedLen
        error('Chromosome length mismatch: expected %d, got %d', expectedLen, length(chrom));
    end
    idx = 1;
    W1v = chrom(idx:idx+lenW1-1); idx = idx + lenW1;
    W2v = chrom(idx:idx+lenW2-1); idx = idx + lenW2;
    W3v = chrom(idx:idx+lenW3-1);

    W1 = reshape(W1v, n_h1, n_input);
    W2 = reshape(W2v, n_h2, n_h1);
    W3 = reshape(W3v, 1, n_h2);

    x0 = zeros(npoles, 1);
    x0(1) = 38.5;

    [tt,y,dy,w,e,de,u,usat, du,~] = sim_ncFF_VIR25(W1, W2, W3, Gs, ref, t, x0, umin, umax, [], is_noise, noise_K,A_tilde,B_tilde,C_tilde,Q,R,P,x_hat);


    IAE = sum(abs(e)) * dt; % Integral of Absolute Error
    IADE = sum(abs(de)) * dt; % Integral of Absolute Derivative of Error
    IUOUTBOUNDS = sum(u < umin | u > umax); % Integral of Control input out of the bounds set by the real system
    IADU = sum(abs(du));

    fit(i) = c1 * IAE + c2 * IADE + c3 * IUOUTBOUNDS + c4 * IADU;
    if isnan(fit(i))
        fit(i) = Inf; % Assign a large fitness value if NaN
    end
end

end