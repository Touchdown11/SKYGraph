%% RUNSTAGE4VISUALIZATION Simulate Stage 3, validate, and show 3-D playback
clc;
close all;

% This file lives in <projectRoot>/scripts. Add every module (src, config,
% models) so all stage functions and the Stage 3 model are on the path.
scriptFolder = fileparts(mfilename('fullpath'));
projectFolder = fileparts(scriptFolder);
addpath(genpath(char(projectFolder)));

modelName = "quadrotor_stage3";
if isempty(which(modelName+".slx")) && ~isfile(modelName+".slx")
    error(['quadrotor_stage3.slx is not in the MATLAB Current Folder or path. ', ...
        'Place the Stage 4 files beside the working Stage 3 model.']);
end

% Reset persistent sensor and estimator states for a repeatable run.
clear sensorModelStage3 stateEstimatorStage3

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
