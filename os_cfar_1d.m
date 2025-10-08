function [thresh, detMask] = os_cfar_1d(x, Ktrain, Gguard, qQuantile)
% Simple OS-CFAR on a 1D vector x (range power).
% For each cell, use Ktrain training cells on each side (excluding Gguard),
% estimate noise as the q-th quantile of the training set, and set thresh
% as a scalar multiple (here 1.0; tune to hit desired Pfa).

    N = numel(x);
    detMask = false(N,1);
    thresh  = zeros(N,1);

    for i=1:N
        L1 = max(1, i - Gguard - Ktrain) : max(1, i - Gguard - 1);
        R1 = min(N, i + Gguard + 1) : min(N, i + Gguard + Ktrain);
        train = [x(L1); x(R1)];
        if numel(train) < 8, thresh(i) = Inf; continue; end
        t = quantile(train, qQuantile);
        
        % Scale factor for Pfa can be tuned; start with 1.5×
        thr = 1.5 * t;
        thresh(i) = thr;
        detMask(i) = x(i) > thr;
    end
end
