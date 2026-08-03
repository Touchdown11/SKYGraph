%% RUNSTAGE4VISUALIZATION Simulate Stage 3, validate, and show 3-D playback
clc;
close all;

modelName = "C:\Users\ASUS\MATLAB\drone_from_scratch\drone_stage_3\quadrotor_stage3";
if isempty(which(modelName+".slx")) && ~isfile(modelName+".slx")
    error(['quadrotor_stage3.slx is not in the MATLAB Current Folder or path. ', ...
        'Place the Stage 4 files beside the working Stage 3 model.']);
end

% Reset persistent sensor and estimator states for a repeatable run.
clear C:\Users\ASUS\MATLAB\drone_from_scratch\drone_stage_3\sensorModelStage3 C:\Users\ASUS\MATLAB\drone_from_scratch\drone_stage_3\stateEstimatorStage3

fprintf('Running %s...\n',modelName);
out = sim(modelName);

% Stop before animation if logging or state data is invalid.
stage4Report = validateStage4Integration(out); %#ok<NASGU>

% CameraMode choices: 'fixed', 'chase', or 'top'.
animateDroneStage4(out, ...
    'PlaybackSpeed',1.5, ...
    'FrameRate',30, ...
    'CameraMode','fixed', ...
    'ShowEstimate',true);
