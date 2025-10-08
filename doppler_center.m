function y0 = doppler_center(y)
    % Estimate bulk Doppler and remove it by complex rotation.
    % y: Nd x 1 slow-time complex series
    Nd = numel(y);
    Y = fftshift(fft(y));
    [~,kpk] = max(abs(Y));
    fbin = kpk - floor(Nd/2) - 1;     % bin offset from DC
    phi = -2*pi*fbin*(0:Nd-1)/Nd;
    y0 = y(:) .* exp(1j*phi(:));
end
