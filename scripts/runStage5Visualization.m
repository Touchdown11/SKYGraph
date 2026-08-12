%% RUNSTAGE5VISUALIZATION Simulate, validate, and animate waypoint mission
clc;
close all;

% This file lives in <projectRoot>/scripts. Add every module (src, config,
% models, tests) so all stage functions are available regardless of layout.
stage5Folder = fileparts(mfilename('fullpath'));
projectFolder = fileparts(stage5Folder);
addpath(genpath(char(projectFolder)));

requiredFunctions = [ ...
    "waypointTrajectoryStage4"; ...
    "flightControllerStage4Waypoints"; ...
    "sensorModelStage3"; ...
    "stateEstimatorStage3"; ...
    "quadDynamicsStage3"; ...
    "animateWaypointMissionStage5"];
for functionIndex = 1:numel(requiredFunctions)
    if isempty(which(requiredFunctions(functionIndex)))
        error('Required function is not on the MATLAB path: %s', ...
            requiredFunctions(functionIndex));
    end
end

modelName = "quadrotor_stage4_waypoints";
modelFile = which(modelName+".slx");
if isempty(modelFile)
    error(['quadrotor_stage4_waypoints.slx is not on the MATLAB path. ', ...
           'Add the folder containing the working waypoint model.']);
end
fprintf('Using waypoint model: %s\n',modelFile);

clear sensorModelStage3 stateEstimatorStage3
out = sim(modelName);
stage5Report = validateStage5Integration(out);
if ~stage5Report.Passed
    error('Stage 5 validation failed. Correct the reported issue before animation.');
end

animateWaypointMissionStage5(out, ...
    'PlaybackSpeed',1.7, ...
    'FrameRate',30, ...
    'CameraMode','fixed', ...
    'ShowEstimate',true, ...
    'ShowGPS',true);
