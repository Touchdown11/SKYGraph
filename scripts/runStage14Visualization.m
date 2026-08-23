function [out,report]=runStage14Visualization()
%RUNSTAGE14VISUALIZATION Simulate Stage 13 then validate/animate Stage 14.
%
% This is the canonical entry point for the final SkyGraph mission
% dashboard. It configures the full pipeline (configureStage13PPO adds every
% module under <projectRoot>/src via genpath), simulates the Stage 13 model,
% validates the required signal logs, and plays the Stage 14 dashboard.

% This file lives in <projectRoot>/scripts, so its parent is the project root.
scriptFolder=fileparts(mfilename('fullpath'));
projectRoot=fileparts(scriptFolder);

% Bootstrap: make every module (src, config, models, tests) visible even if
% the project root was never added to the MATLAB path manually.
addpath(genpath(char(projectRoot)));

% configureStage13PPO resolves the project root from its own location under
% <projectRoot>/config and (re)adds all modules/models/tests to the path.
configureStage13PPO(projectRoot);

% Clear persistent state so every run is repeatable.
clear gatThreatInferenceStage12 realisticScenarioToFStage8 entityTrackerStage9
clear sensorModelStage3 stateEstimatorStage3

out=sim("quadrotor_stage13_ppo");

% Validate all signal logs required by the dashboard.
report=validateSkyGraphStage14(out);

% Play the final 3-D mission dashboard (fixed camera for a clean render).
animateSkyGraphStage14(out,'PlaybackSpeed',2,'FrameRate',25,'CameraMode','fixed');
end
