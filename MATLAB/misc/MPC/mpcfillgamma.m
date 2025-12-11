function Gamma = mpcfillgamma(n_outs, pred)
    I = eye(n_outs, n_outs);
    Gamma = tril(repmat(I, pred, pred));
end