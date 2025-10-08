function [TFbest, alphaStar] = stfrft_order_scan(y, alphaGrid)
% Single-window short-time FrFT: here "short-time" = one window (Nd=128).
% Returns the TF slice at the best alpha (by peak^2 / energy).
% You can extend to multi-window later if desired.

Nd = numel(y);
bestScore = -Inf;
alphaStar = alphaGrid(1);
TFbest = [];

for a = alphaGrid
    % fractional order mapping: alpha (rad) -> 'p' in FRFT
    p = a/pi;     % common convention (adjust if you prefer)
    Ya = frft(y, p);              % complex spectrum in fractional domain
    TF = abs(Ya).^2;              % energy
    score = max(TF)^2 / (sum(TF) + eps);   % simple concentration metric
    if score > bestScore
        bestScore = score;
        alphaStar = a;
        TFbest = TF;
    end
end
end
