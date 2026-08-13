function configureStage13PPO(projectRoot)
%CONFIGURESTAGE13PPO Add inherited paths, load PPO and configure model.
%
% configureStage13PPO("C:\Users\ASUS\MATLAB\drone_from_scratch")

stage13Folder=fileparts(mfilename('fullpath'));
if nargin<1
    projectRoot=fileparts(stage13Folder);
else
    projectRoot=string(projectRoot);
end
if ~isfolder(projectRoot),error('Project root does not exist: %s',projectRoot);end
addpath(genpath(char(projectRoot)));
addpath(stage13Folder,'-begin');
rehash;

required=["skyGraphScenarioStage8";"realisticScenarioToFStage8"; ...
    "entityTrackerStage9";"graphBuilderStage10"; ...
    "nominalAccelerationStage11";"cbfSafetyShieldStage11"; ...
    "attitudeMotorControllerStage11";"gatThreatInferenceStage12"; ...
    "ppoObservationStage13";"policySelectorStage13"];
missing=strings(0,1);
fprintf('\nSTAGE 13 SOURCE CHECK\n');
for k=1:numel(required)
    location=which(required(k));
    if isempty(location)
        missing(end+1)=required(k); %#ok<AGROW>
        fprintf('MISSING  %s\n',required(k));
    else
        fprintf('OK       %-36s %s\n',required(k),location);
    end
end
if ~isempty(missing)
    error('Missing Stage 13 dependencies: %s',strjoin(missing,', '));
end

agentFile=fullfile(stage13Folder,'ppoAgentStage13.mat');
if ~isfile(agentFile)
    matches=dir(fullfile(char(projectRoot),'**','ppoAgentStage13.mat'));
    if isempty(matches)
        error(['ppoAgentStage13.mat was not found. This trained agent is generated at ', ...
            'runtime (it is gitignored as *.mat). Run setupSkyGraph once to train the ', ...
            'Stage 12 GAT weights and the Stage 13 PPO agent before running this stage.']);
    end
    agentFile=fullfile(matches(1).folder,matches(1).name);
end
loaded=load(agentFile,'agent');loaded.agent.UseExplorationPolicy=false;
assignin('base','ppoAgentStage13',loaded.agent);

model="quadrotor_stage13_ppo";
modelFile=which(model+".slx");
if isempty(modelFile)
    matches=dir(fullfile(char(projectRoot),'**',model+".slx"));
    if isempty(matches),error('%s.slx was not found.',model);end
    modelFile=fullfile(matches(1).folder,matches(1).name);
    addpath(matches(1).folder,'-begin');
end
load_system(modelFile);
observationConfig=get_param(model+"/Stage 13 PPO Observation", ...
    "MATLABFunctionConfiguration");
observationConfig.UpdateMethod="Discrete";observationConfig.SampleTime="0.10";
selectorConfig=get_param(model+"/Stage 13 Policy Selector", ...
    "MATLABFunctionConfiguration");
selectorConfig.UpdateMethod="Discrete";selectorConfig.SampleTime="0.05";
set_param(model,'SolverType','Fixed-step','Solver','ode4', ...
    'FixedStep','0.005','StopTime','30');
save_system(model);
fprintf('Loaded PPO agent and configured Stage 13 model: %s\n',modelFile);
end
