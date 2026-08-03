function [out,report]=runStage14Visualization()
%RUNSTAGE14VISUALIZATION Simulate Stage 13 then validate/animate Stage 14.

stage14Folder=fileparts(mfilename('fullpath'));
projectRoot=fileparts(stage14Folder);
setupStage12Paths(projectRoot);
addpath(fullfile(projectRoot,'skygraph_stage13_ppo'),'-begin');
addpath(stage14Folder,'-begin');
configureStage13PPO;
clear gatThreatInferenceStage12 realisticScenarioToFStage8 entityTrackerStage9
clear sensorModelStage3 stateEstimatorStage3
out=sim("quadrotor_stage13_ppo");
report=validateSkyGraphStage14(out);
animateSkyGraphStage14(out,'PlaybackSpeed',2,'FrameRate',25,'CameraMode','fixed');
end
