function [output,metrics]=runMultiEgoStage16(configuration)
%RUNMULTIEGOSTAGE16 Public entry point for the decentralized multi-ego demo.
root=fileparts(fileparts(mfilename('fullpath'))); addpath(genpath(root));
if nargin<1,configuration=configureMultiEgoStage16();end
if isfield(configuration,'Visualize') && configuration.Visualize, configuration.Visualize=false; showDashboard=true; else, showDashboard=false; end
output=runMultiEgoSimulationStage16(configuration); metrics=extractMultiEgoMetricsStage16(output);
dataFolder=fullfile(root,'data'); writetable(metrics,fullfile(dataFolder,'stage16_multiego_summary.csv'));
save(fullfile(dataFolder,'stage16_multiego_results.mat'),'output','metrics');
disp(metrics);
if showDashboard,animateMultiEgoSkyGraphDashboardStage14(output,'PlaybackSpeed',2,'FrameRate',20);end
end
