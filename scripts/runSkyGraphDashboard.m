function [out, report] = runSkyGraphDashboard(out)
%RUNSKYGRAPHDASHBOARD Run the Stage 14 SkyGraph Mission Dashboard end-to-end.
%
%   runSkyGraphDashboard()      - train any missing artifacts, configure the
%                                 pipeline, simulate Stage 13, validate the
%                                 logs, then play the multi-panel dashboard.
%   runSkyGraphDashboard(out)   - play the dashboard using existing simulation
%                                 output (e.g. from runStage14Visualization).
%
% This is the simplest way to "just see the dashboard". Playback controls:
%   Space = pause/resume   Esc = stop.
%
% See also runStage14Visualization, setupSkyGraph.

% This file lives in <projectRoot>/scripts, so its parent is the project root.
scriptFolder = fileparts(mfilename('fullpath'));
projectRoot   = fileparts(scriptFolder);
addpath(genpath(char(projectRoot)));

if nargin < 1 || isempty(out)
    % No simulation output supplied: ensure the trained artifacts exist,
    % configure the pipeline, and simulate the Stage 13 model.
    setupSkyGraph();
    configureStage13PPO(projectRoot);

    % Clear persistent state so every run is repeatable.
    clear gatThreatInferenceStage12 realisticScenarioToFStage8 entityTrackerStage9
    clear sensorModelStage3 stateEstimatorStage3

    out = sim("quadrotor_stage13_ppo");
end

% Validate all signal logs required by the dashboard.
report = validateSkyGraphStage14(out);

% Play the multi-panel SkyGraph mission dashboard.
animateSkyGraphDashboard(out, 'PlaybackSpeed', 2, 'FrameRate', 20);
end
