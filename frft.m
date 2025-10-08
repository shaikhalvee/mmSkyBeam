function X = frft(x, a)
% FRFT  Fractional Fourier Transform of order 'a' (a real scalar).
% Simple O(N^2) implementation adequate for N=128.
x = x(:);
N = numel(x);
if mod(N,2)==0
    n = (-N/2:(N/2-1)).';
else
    n = (-(N-1)/2:(N-1)/2).';
end
alpha = a*pi/2;  % map to rotation angle
if mod(a,2)==0
    % ordinary FT multiples: quick exits
    switch mod(round(a),4)
        case 0, X = x;
        case 1, X = fftshift(fft(ifftshift(x)));
        case 2, X = flipud(x);
        case 3, X = fftshift(ifft(ifftshift(x)));
    end
    return;
end
cotA = cot(alpha); cscA = csc(alpha);

% Chirp multiplication
x1 = exp(-1j*pi*cotA*(n.^2)/N) .* x;

% Chirp convolution (direct O(N^2); OK for N=128)
h  = exp( 1j*pi*cscA*( (n).^2 )/N );
X1 = zeros(N,1);
for k=1:N
    X1(k) = sum(x1 .* circshift(h, k-1));
end

% Final chirp
X = sqrt(1 - 1j*cotA)/sqrt(N) * exp(-1j*pi*cotA*(n.^2)/N) .* X1;
end
