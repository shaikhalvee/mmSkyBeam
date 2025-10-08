% RUN_UAV_TXBF_DETECTION.M
% End-to-end pipeline for camera-steered TX beamforming + LCMV + sST-FrFT.
% Assumes:
%   - 9-TX ULA (codebook steering per angle)
%   - 16-RX (non-uniform 0.5λ positions, provided as params.D_RX)
%   - 512 ADC samples, 128 chirps
%   - RX phase calibration vector BF_MIMO_ref (degrees) available

close all; clc; clearvars;

%% ----------------- USER CONFIG ---------------------------------------
adc_data_folder = 'G:\RADAR_DATA\out_txbf_25_8_sl_15_fr_200';
[~, testRootFolder, ~] = fileparts(adc_data_folder);
output_folder  = ['./output/' testRootFolder];
oldParamsFile  = [output_folder filesep testRootFolder '_params.mat'];
frame_folder   = [output_folder filesep 'rangeDopplerFFTmap_11/'];
calib_file     = './input/calibrConfig/calibrateResults_dummy.mat';

if ~exist(frame_folder,'dir'), mkdir(frame_folder); end

%% ----------------- LOAD PARAMS & CALIB -------------------------------
oldParams = load(oldParamsFile, 'params');
params    = oldParams.params;

% Refresh fields from recorded binary metadata
configFromAdcData = get_radar_config(params.anglesToSteer, [adc_data_folder '\']);
params.Slope_MHzperus   = configFromAdcData.Slope_MHzperus;
params.TI_Cascade_RX_ID = configFromAdcData.TI_Cascade_RX_ID;
params.numRX            = configFromAdcData.numRX;
params.D_RX             = configFromAdcData.D_RX;    % RX pos in 0.5λ units (non-uniform ULA)
params.calibrationInterp= configFromAdcData.calibrationInterp;
params.phaseCalibOnly   = configFromAdcData.phaseCalibOnly;

% Fixed constraints you asked me to remember
numAdc = params.Samples_per_Chirp;
numChirp = params.nchirp_loops;
numRx = params.numRX;
numAngle = params.NumAnglesToSweep;

% Optional clutter handling
opts.dcOffsetRemoval     = true;
opts.dopplerClutterRemoval = true;

% RX phase calib (deg)
load(calib_file, 'calibResult');
BF_MIMO_ref_deg = calibResult.RxMismatch(:);   % length = numRX

%% ----------------- READ INDEX & PROCESS FRAMES -----------------------
fileIdx_unique = getUniqueFileIdx(adc_data_folder);
all_detections = [];    % [frame, angleIdx, range_m, score]

% all_RD_map = {};        % cell array for all frames (if #frames can differ per file)
all_range_axis = {};
all_doppler_axis = {};
% all_range_angle_stich = {};
% all_to_plot = {};

frameCounter = 1;

for i_file = 1:numel(fileIdx_unique)
    fileNameStruct = getBinFileNames_withIdx(adc_data_folder, fileIdx_unique{i_file});
    [numValidFrames, ~] = getValidNumFrames(fullfile(adc_data_folder, fileNameStruct.masterIdxFile));

    for frameId = 2:numValidFrames
        params.frameId = frameId;

        % ---- LOAD RAW ADC CUBE: [Nsamp, Nchirp, Nrx, Nangle] ----------
        adcCube = fcn_read_AdvFrmConfig_BF_Json(fileNameStruct, params);
        % Apply TI channel reorder
        adcCube = adcCube(:,:,params.TI_Cascade_RX_ID,:);

        % ---- PROCESS THIS FRAME (does everything through detection) ----
        results = process_frame_uav(adcCube, params, BF_MIMO_ref_deg, opts);

        % Save per-frame RD (optional & large)
        save(fullfile(frame_folder, sprintf('frame_%05d.mat', frameCounter)), ...
             '-struct','results','RD_beam','range_axis','doppler_axis','detections','-v7.3');

        % Collect detections
        if ~isempty(results.detections)
            all_detections = [all_detections; [repmat(frameCounter,size(results.detections,1),1), results.detections]];
        end

        all_range_axis{frameCounter} = range_axis;

        fprintf('[INFO] processed frame %d (%d/%d in file %d)\n', frameCounter, frameId, numValidFrames, i_file);
        frameCounter = frameCounter + 1;
    end
end

save(fullfile(output_folder, "all_detections.mat"), "all_detections", '-v7.3');
