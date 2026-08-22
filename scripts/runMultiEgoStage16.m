function [output,metrics]=runMultiEgoStage16(configuration)
%RUNMULTIEGOSTAGE16 Public entry point for the decentralized multi-ego demo.
root=fileparts(fileparts(mfilename('fullpath'))); addpath(genpath(root));
if nargin<1,configuration=configureMultiEgoStage16();end
output=runMultiEgoSimulationStage16(configuration); metrics=extractMultiEgoMetricsStage16(output);
dataFolder=fullfile(root,'data'); writetable(metrics,fullfile(dataFolder,'stage16_multiego_summary.csv'));
save(fullfile(dataFolder,'stage16_multiego_results.mat'),'output','metrics');
disp(metrics);
end
