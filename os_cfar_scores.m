function [thr, detMask] = os_cfar_scores(scores, Ktrain, Gguard, qQuantile)
    % OS-CFAR over a SHORT list of gate scores (robust thresholding)
    N = numel(scores);
    detMask = false(N,1);
    thr = zeros(N,1);
    for i=1:N
        L1 = max(1, i - Gguard - Ktrain) : max(1, i - Gguard - 1);
        R1 = min(N, i + Gguard + 1) : min(N, i + Gguard + Ktrain);
        train = [scores(L1); scores(R1)];
        if numel(train) < 4, thr(i) = Inf; continue; end
        t = quantile(train, qQuantile);
        thr(i) = 1.2 * t;                 % slightly gentler than range CFAR
        detMask(i) = scores(i) > thr(i);
    end
end
