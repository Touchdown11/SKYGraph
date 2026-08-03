function configureStage15Evaluation(projectRoot)
%CONFIGURESTAGE15EVALUATION Add project paths, load PPO, configure model.
%
% Use either:
%   configureStage15Evaluation
% or:
%   configureStage15Evaluation("C:\Users\...\drone_from_scratch")

stage15Folder=fileparts(mfilename('fullpath'));
if nargin<1
    % If this file is in its own stage folder, use its parent as project
    % root. If it is already in the project root, use its own folder.
    parentFolder=fileparts(stage15Folder);
    if isfolder(fullfile(parentFolder,'skygraph_stage12_gat')) ...
            || isfolder(fullfile(parentFolder,'skygraph_stage13_ppo'))
        projectRoot=parentFolder;
    else
        projectRoot=stage15Folder;
    end
else
    projectRoot=string(projectRoot);
end
if ~isfolder(projectRoot)
    error('Project root does not exist: %s',projectRoot);
end

% Add every stage folder below the selected project root. This removes the
% dependency on setupStage12Paths.m and also supports a flat project folder.
addpath(genpath(char(projectRoot)));
addpath(stage15Folder,'-begin');
rehash;

requiredFunctions=[ ...
    "quadDynamicsStage3"; ...
    "sensorModelStage3"; ...
    "stateEstimatorStage3"; ...
    "waypointTrajectoryStage4"; ...
    "realisticScenarioToFStage8"; ...
    "entityTrackerStage9"; ...
    "graphBuilderStage10"; ...
    "nominalAccelerationStage11"; ...
    "cbfSafetyShieldStage11"; ...
    "attitudeMotorControllerStage11"; ...
    "gatThreatInferenceStage12"; ...
    "ppoObservationStage13"; ...
    "policySelectorStage13"; ...
    "safetySelectorStage15"];

fprintf('\nSTAGE 15 SOURCE CHECK\n');
missing=strings(0,1);
for k=1:numel(requiredFunctions)
    location=which(requiredFunctions(k));
    if isempty(location)
        missing(end+1)=requiredFunctions(k); %#ok<AGROW>
        fprintf('MISSING  %s\n',requiredFunctions(k));
    else
        fprintf('OK       %-36s %s\n',requiredFunctions(k),location);
    end
end
if ~isempty(missing)
    error(['Required source files are missing: %s. Extract/copy the ', ...
        'corresponding stage packages under %s.'],strjoin(missing,', '),projectRoot);
end

agentFile=which('ppoAgentStage13.mat');
if isempty(agentFile)
    matches=dir(fullfile(char(projectRoot),'**','ppoAgentStage13.mat'));
    if isempty(matches)
        error('ppoAgentStage13.mat was not found under %s.',projectRoot);
    end
    agentFile=fullfile(matches(1).folder,matches(1).name);
    addpath(matches(1).folder,'-begin');
end
loaded=load(agentFile,'agent');
loaded.agent.UseExplorationPolicy=false;
assignin('base','ppoAgentStage13',loaded.agent);
fprintf('Loaded PPO agent: %s\n',agentFile);

modelName="quadrotor_stage15_evaluation";
modelFile=which(modelName+".slx");
if isempty(modelFile)
    matches=dir(fullfile(char(projectRoot),'**',modelName+".slx"));
    if isempty(matches)
        error('%s.slx was not found under %s.',modelName,projectRoot);
    end
    modelFile=fullfile(matches(1).folder,matches(1).name);
    addpath(matches(1).folder,'-begin');
end
load_system(modelFile);
configuration=get_param(modelName+"/Stage 15 Safety Selector", ...
    "MATLABFunctionConfiguration");
configuration.UpdateMethod="Discrete";
configuration.SampleTime="0.05";
set_param(modelName,'SolverType','Fixed-step','Solver','ode4', ...
    'FixedStep','0.005','StopTime','30');
save_system(modelName);
fprintf('Stage 15 evaluation model configured: %s\n',modelFile);
end
