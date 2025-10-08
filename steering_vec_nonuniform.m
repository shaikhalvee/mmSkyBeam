function a = steering_vec_nonuniform(posHalfLambda, thetaRad)
    % posHalfLambda: column vector of RX positions in units of 0.5 λ
    % thetaRad: azimuth (broadside = 0) in radians
    a = exp(1j*pi*posHalfLambda(:)*sin(thetaRad));
end
