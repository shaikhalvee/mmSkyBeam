function score = presence_score_concentration(TF, topFrac)
    % Concentration-gain: energy in top-K% pixels / total energy
    v = TF(:);
    E = sum(v);
    K = max(1, round(topFrac * numel(v)));
    sv = sort(v, 'descend');
    score = sum(sv(1:K)) / (E + eps);
end
