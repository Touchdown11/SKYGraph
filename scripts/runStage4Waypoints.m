%% RUNSTAGE4WAYPOINTS Run, validate, and plot smooth waypoint flight
clc;
close all;

% This file lives in <projectRoot>/scripts. Add every module (src, config,
% models, tests) so all stage functions are available regardless of layout.
scriptFolder = fileparts(mfilename('fullpath'));
projectFolder = fileparts(scriptFolder);
addpath(genpath(char(projectFolder)));

requiredFunctions = [ ...
    "waypointTrajectoryStage4"; ...
    "flightControllerStage4Waypoints"; ...
    "positionErrorStage4Waypoints"; ...
    "environmentStage4Waypoints"; ...
    "sensorModelStage3"; ...
    "stateEstimatorStage3"; ...
    "quadDynamicsStage3"];
for k = 1:numel(requiredFunctions)
    if isempty(which(requiredFunctions(k)))
        error('Required function is not on the MATLAB path: %s', ...
            requiredFunctions(k));
    end
end

modelName = "quadrotor_stage4_waypoints";
modelFile = which(modelName+".slx");
if isempty(modelFile) && isfile(modelName+".slx")
    modelFile = fullfile(pwd,modelName+".slx");
end
if isempty(modelFile)
    error(['quadrotor_stage4_waypoints.slx was not found. ', ...
        'Create it by copying the working Stage 3 model.']);
end

fprintf('Using waypoint model: %s\n',modelFile);
clear sensorModelStage3 stateEstimatorStage3
out = sim(modelName);
waypointReport = validateStage4Waypoints(out); %#ok<NASGU>
plotStage4Waypoints
