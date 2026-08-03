%% RUNSKYGRAPHDASHBOARD Validate current output and start dashboard
if ~exist('out','var')
    error(['Variable out was not found. Run the Stage 13/15 model first, ', ...
        'for example: out = sim("quadrotor_stage13_ppo");']);
end
validateSkyGraphStage14(out);
animateSkyGraphDashboard(out,'PlaybackSpeed',2,'FrameRate',20);
