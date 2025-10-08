function TFenh = sparse_soft_threshold(TF)
    % Soft-threshold in TF domain with universal threshold τ = σ̂ sqrt(2 log N)
    % TF can be a vector or 2D map; here it's a vector (single-window)
    
    v = TF(:);
    
    % Robust noise estimate via MAD on lower half of magnitudes
    medv = median(v);
    madv = median(abs(v - medv)) / 0.6745;
    sigma_hat = max(madv, eps);
    tau = sigma_hat * sqrt(2*log(numel(v)));

    % Soft shrinkage on magnitudes
    mag = v;
    mag = max(mag - tau, 0);
    TFenh = reshape(mag, size(TF));
end
