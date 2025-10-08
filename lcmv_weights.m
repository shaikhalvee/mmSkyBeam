function w = lcmv_weights(R, C, f)
% LCMV weights: w = R^{-1} C (C^H R^{-1} C)^{-1} f
% Inputs:
%   R: N x N covariance (Hermitian, loaded)
%   C: N x K constraint steering matrix
%   f: K x 1 desired response vector

    RiC = R \ C;
    M   = (C' * RiC);
    w   = RiC * (M \ f);
end
