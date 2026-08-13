function [weightsFile, agentFile] = setupSkyGraph()
%SETUPSKYGRAPH Train the runtime artifacts needed by Stage 14 & 15.
%
% The final pipeline (Stage 14 visualization and Stage 15 evaluation) loads
% two trained artifacts that are generated at runtime and therefore are NOT
% stored in the repository (they are gitignored as *.mat):
%
%   1. gatThreatWeightsStage12.mat  - GAT threat encoder weights (Stage 12)
%   2. ppoAgentStage13.mat          - PPO navigation policy agent (Stage 13)
%
% Run this once after cloning, BEFORE calling runStage14Visualization or
% runStage15Experiments:
%
%    addpath(genpath(pwd));
%    [weightsFile, agentFile] = setupSkyGraph();
%
% The function trains Stage 12 first (Stage 13 depends on it), then Stage 13.
% It skips any artifact that already exists on the MATLAB path.

scriptFolder = fileparts(mfilename('fullpath'));
projectRoot  = fileparts(scriptFolder);
addpath(genpath(char(projectRoot)));

fprintf('\nSKYGRAPH SETUP - checking trained artifacts\n');
fprintf('===========================================\n');

% ---- Stage 12: GAT threat encoder weights --------------------------------
weightsFile = which('gatThreatWeightsStage12.mat');
if isempty(weightsFile)
    fprintf('Missing gatThreatWeightsStage12.mat -> training Stage 12 GAT...\n');
    trainGATThreatStage12();
    weightsFile = which('gatThreatWeightsStage12.mat');
    if isempty(weightsFile)
        weightsFile = fullfile(scriptFolder,'gatThreatWeightsStage12.mat');
    end
    fprintf('Stage 12 GAT weights ready: %s\n', weightsFile);
else
    fprintf('OK  gatThreatWeightsStage12.mat found at: %s\n', weightsFile);
end

% ---- Stage 13: PPO agent -------------------------------------------------
agentFile = which('ppoAgentStage13.mat');
if isempty(agentFile)
    fprintf('Missing ppoAgentStage13.mat -> training Stage 13 PPO...\n');
    trainPPOStage13();
    agentFile = which('ppoAgentStage13.mat');
    if isempty(agentFile)
        agentFile = fullfile(scriptFolder,'ppoAgentStage13.mat');
    end
    fprintf('Stage 13 PPO agent ready: %s\n', agentFile);
else
    fprintf('OK  ppoAgentStage13.mat found at: %s\n', agentFile);
end

fprintf('SETUP COMPLETE. You can now run runStage14Visualization / runStage15Experiments.\n\n');
end
