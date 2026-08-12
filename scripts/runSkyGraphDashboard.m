%% RUNSKYGRAPHDASHBOARD Validate current output and start dashboard
% Add every module (src, config, models, tests) so the validation and
% dashboard functions are found regardless of the current MATLAB folder.
addpath(genpath(fileparts(fileparts(mfilename('fullpath')))));
if ~exist('out','var')
    error(['Variable out was not found. Run the Stage 13/15 model first, ', ...
        'for example: out = sim("quadrotor_stage13_ppo");']);
end
validateSkyGraphStage14(out);
animateSkyGraphDashboard(out,'PlaybackSpeed',2,'FrameRate',20);
