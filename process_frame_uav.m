function out = process_frame_uav(adcRadarData_txbf, params, BF_MIMO_ref_deg, opts)
% PROCESS_FRAME_UAV
% Runs: Range FFT → RX cal → (per-range) LCMV → Doppler FFT → CFAR gates
% → slow-time extract + Doppler-center → sST-FrFT α-scan → soft-threshold
% → presence score → OS-CFAR decision.
%
% Inputs:
%   adcCube: [Nsamp, Nchirp, Nrx, Nangle]
%   params:  struct from your config (must contain D_RX positions, angles)
%   BF_MIMO_ref_deg: per-Rx phase calib (deg), length = Nrx
%   opts:    .dcOffsetRemoval, .dopplerClutterRemoval (bool)
%
% Output:
%   out.RD_beam{angle}: [Nrange x Ndoppler] beamformed RD map per angle
%   out.range_axis, out.doppler_axis
%   out.detections: [angleIdx, range_m, score, decision]

c  = 3e8;
fs = params.Sampling_Rate_ksps * 1e3;               % 16e6
S  = params.Slope_MHzperus * 1e12;                  % Hz/s
T_chirp = (params.Idle_Time_us + params.Ramp_End_Time_us)*1e-6; % 45 us
lambda  = c / (params.Start_Freq_GHz*1e9);

Ns = params.Samples_per_Chirp;
Nd = params.nchirp_loops;
Nr = 2^nextpow2(Ns);
Nd_fft = 2^nextpow2(Nd);

Nrx = params.numRX;
Nang = params.NumAnglesToSweep;
pos  = params.D_RX(:);   % positions in 0.5λ units (non-uniform ULA)

% Windowing
% w_range   = hann(Ns, 'periodic');
% w_dopp    = hann(Nd, 'periodic');
w_range = hann_local(Ns);
w_dopp = hann_local(Ns);

% Precompute axes
range_bin = (0:Nr-1);
range_axis = range_bin * c*fs/(2*S*Nr);
v_max = lambda/(4*T_chirp);
doppler_axis = linspace(-v_max, v_max, Nd_fft).';

% Outputs
out.RD_beam = cell(Nang,1);
out.range_axis = range_axis;
out.doppler_axis = doppler_axis;
detections = [];

% ----------------- Range FFT & RX calibration (pre-Doppler) -----------
% DC removal in fast-time if requested
if opts.dcOffsetRemoval
    adcRadarData_txbf = adcRadarData_txbf - mean(adcRadarData_txbf,1);
end

% Apply range window
adcCube_win = adcRadarData_txbf .* reshape(w_range,[],1,1,1);

% Range FFT (dim 1)
rangeFFT = fft(adcCube_win, Nr, 1); % [Nr, Nd, Nrx, Nang]

% RX phase-only calibration (apply HERE)
cal = exp(-1j*BF_MIMO_ref_deg(:)*pi/180).'; % row 1xNrx
rangeFFT = rangeFFT .* reshape(cal, 1,1,Nrx,1);

% Optional slow-time mean removal per [r,rx,ang]
if opts.dopplerClutterRemoval
    rangeFFT = rangeFFT - mean(rangeFFT, 2);
end

% Doppler window (apply before making X_r matrices)
rangeFFT = rangeFFT .* reshape(w_dopp,1,[],1,1);

% ----------------- Adaptive RX beamforming (per angle, per range) -----
% We produce a beamformed slow-time per (r,angle), then Doppler FFT it.

delta_loading = 0.05;      % diagonal loading factor (× tr(R)/N)
useFB = true;              % forward-backward averaging flag

for ia = 1:Nang
    theta0_deg = params.anglesToSteer(ia);
    theta0 = deg2rad(theta0_deg);

    % Dynamic TX grating-lobe angles for d_TX = 2λ
    s0 = sin(theta0);
    cand = [s0-0.5, s0+0.5];
    vis  = cand(abs(cand)<=1);
    th_g = asin(vis);           % radians, 0..2 visible lobes

    % Pre-allocate
    Y = complex(zeros(Nr, Nd)); % beamformed slow-time per range

    % Loop ranges: (vectorization across r is possible; clarity first)
    for r = 1:Nr
        Xr = squeeze(rangeFFT(r,:,:,ia)); % [Nd x Nrx]
        if ~any(Xr(:)), continue; end

        % Sample covariance across slow-time
        R = (Xr' * Xr) / Nd;
        R = 0.5*(R+R');                    % symmetrize

        % Forward-backward averaging (optional)
        if useFB
            J = fliplr(eye(Nrx));
            R = 0.5*(R + J*conj(R)*J);
        end

        % Diagonal loading
        R = R + (delta_loading * trace(R)/Nrx) * eye(Nrx);

        % Constraints: unity at θ0, nulls at θg
        C = steering_vec_nonuniform(pos, theta0);
        if ~isempty(th_g)
            for k=1:numel(th_g)
                C = [C, steering_vec_nonuniform(pos, th_g(k))];
            end
        end
        f = [1; zeros(size(C,2)-1,1)];

        % LCMV weights
        w = lcmv_weights(R, C, f);  % Nrx x 1

        % Beamform slow-time
        Y(r,:) = (Xr * w).';        % 1 x Nd
    end

    % Doppler FFT per range
    Yw = Y; % already windowed in slow-time
    RD = fftshift(fft(Yw, Nd_fft, 2), 2);    % [Nr x Nd_fft]
    out.RD_beam{ia} = RD;
end

% ----------------- Gate selection (low-threshold CFAR near DC) --------
% Integrate power around small Doppler band near zero (e.g., ±1 bins)
dcBand = 1;   % +/- 1 bin
gateMaxPerAngle = 4; % 10 range bins

gates = [];
for ia=1:Nang
    RDpow = abs(out.RD_beam{ia}).^2;           % [Nr x Nd_fft]
    dcIdx = floor(Nd_fft/2)+1;
    sl = max(1,dcIdx-dcBand):min(Nd_fft, dcIdx+dcBand);
    rangePow = sum(RDpow(:,sl),2);             % integrate around DC

    % Simple 1D OS-CFAR along range
    Ktrain = 32; Gguard = 4; q = 0.7;
    [thresh, detMask] = os_cfar_1d(rangePow, Ktrain, Gguard, q);

    candIdx = find(detMask);
    if numel(candIdx)>gateMaxPerAngle
        [~,ord] = sort(rangePow(candIdx),'descend');
        candIdx = candIdx(ord(1:gateMaxPerAngle));
    end

    if ~isempty(candIdx)
        gates = [gates; [ia*ones(numel(candIdx),1), candIdx]]; %#ok<AGROW>
    end
end

% ----------------- Per-gate sST-FrFT + sparse threshold + decision ----
detections = [];
if ~isempty(gates)
    alphaGrid = (-0.35*pi):0.02*pi:(0.35*pi);
    for g=1:size(gates,1)
        ia = gates(g,1);  r = gates(g,2);

        % Extract slow-time and Doppler-center
        y = squeeze(out.RD_beam{ia}(r,:)); %#ok<NASGU> % (only has Doppler domain)
        % Re-extract from pre-FFT beamformed slow-time to center properly:
        % (We saved Y per angle in local scope; recompute if needed)
        Xr = squeeze(rangeFFT(r,:,:,ia));
        R  = (Xr' * Xr)/Nd; R = 0.5*(R+R');
        if useFB, J=fliplr(eye(Nrx)); R=0.5*(R+J*conj(R)*J); end
        R = R + (delta_loading * trace(R)/Nrx) * eye(Nrx);
        C = steering_vec_nonuniform(pos, deg2rad(params.anglesToSteer(ia)));
        s0 = sin(deg2rad(params.anglesToSteer(ia)));
        cand = [s0-0.5, s0+0.5]; vis = cand(abs(cand)<=1); th_g = asin(vis);
        if ~isempty(th_g)
            for k=1:numel(th_g), C=[C, steering_vec_nonuniform(pos, th_g(k))]; end
        end
        f = [1; zeros(size(C,2)-1,1)];
        w = lcmv_weights(R,C,f);

        yslow = (Xr * w);                    % Nd x 1 slow-time
        yslow = doppler_center(yslow);       % rotate to center bulk Doppler

        % sST-FrFT (single window = Nd=128) with order scan
        [TFbest, alphaStar] = stfrft_order_scan(yslow, alphaGrid);

        % Sparse soft-threshold
        TFenh = sparse_soft_threshold(TFbest);

        % Presence score (concentration gain, top-K%)
        score = presence_score_concentration(TFenh, 0.02); % top 2%

        % OS-CFAR across gates (build vector of gate scores per angle)
        detections = [detections; ia, range_axis(r), score, 0, alphaStar];
    end

    % Threshold scores with OS-CFAR across the set of gates (per angle)
    if ~isempty(detections)
        for ia = unique(detections(:,1)).'
            mask = detections(:,1)==ia;
            scores = detections(mask,3);
            [thr, detMask] = os_cfar_scores(scores, 10, 2, 0.7);
            dd = detections(mask,:);
            dd(:,4) = detMask;
            detections(mask,:) = dd;
        end
    end
end

out.detections = detections;
end
