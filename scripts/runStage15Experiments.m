function results=runStage15Experiments(suite)
%RUNSTAGE15EXPERIMENTS Run quick (8) or full (40) experiment suite.

if nargin<1,suite="quick";end
suite=lower(string(suite));

% This file lives in <projectRoot>/scripts, so its parent is the project root.
scriptFolder=fileparts(mfilename('fullpath'));
projectRoot=fileparts(scriptFolder);
addpath(genpath(char(projectRoot)));

% Auto-train any missing runtime artifacts (Stage 12 GAT weights and the
% Stage 13 PPO agent) before configuring the evaluation harness.
setupSkyGraph();

configureStage15Evaluation(projectRoot);
model="quadrotor_stage15_evaluation";

% Results are written to the shared <projectRoot>/data folder.
dataFolder=fullfile(projectRoot,'data');

if suite=="full"
    names=strings(40,1);policy=zeros(40,1);safety=zeros(40,1);
    scenario=zeros(40,1);fault=zeros(40,1);index=0;
    for scenarioMode=1:5
        for faultMode=0:1
            for policyMode=0:1
                for safetyMode=0:1
                    index=index+1;policy(index)=policyMode;safety(index)=safetyMode;
                    scenario(index)=scenarioMode;fault(index)=faultMode;
                    names(index)=sprintf('P%d_S%d_SC%d_F%d',policyMode,safetyMode,scenarioMode,faultMode);
                end
            end
        end
    end
else
    names=["WP_noCBF_dense";"WP_CBF_dense";"PPO_noCBF_dense";"PPO_CBF_dense"; ...
           "WP_CBF_blind";"PPO_CBF_blind";"WP_CBF_dense_faults";"PPO_CBF_dense_faults"];
    policy=[0;0;1;1;0;1;0;1];safety=[0;1;0;1;1;1;1;1];
    scenario=[5;5;5;5;3;3;5;5];fault=[0;0;0;0;0;0;1;1];
end
configurations=table(names,policy,safety,scenario,fault, ...
    'VariableNames',{'Name','PolicyMode','SafetyMode','ScenarioMode','FaultSchedule'});
results=table;
for run=1:height(configurations)
    configuration=configurations(run,:);
    fprintf('\n[%d/%d] %s\n',run,height(configurations),configuration.Name);
    clear realisticScenarioToFStage8 entityTrackerStage9 gatThreatInferenceStage12
    input=Simulink.SimulationInput(model);
    input=input.setBlockParameter(model+"/Stage 13 Policy Mode", ...
        'Value',num2str(configuration.PolicyMode));
    input=input.setBlockParameter(model+"/Stage 15 Safety Mode", ...
        'Value',num2str(configuration.SafetyMode));
    input=input.setBlockParameter(model+"/Stage 8 Scenario Mode", ...
        'Value',num2str(configuration.ScenarioMode));
    input=input.setBlockParameter(model+"/Stage 8 Fault Schedule Enabled", ...
        'Value',num2str(configuration.FaultSchedule));
    try
        output=sim(input);
        row=extractStage15Metrics(output,configuration);
        row.RunSucceeded=true;
        row.ErrorMessage="";
    catch exception
        warning('Stage15:RunFailed','%s failed: %s',configuration.Name,exception.message);
        row=failureRow15(configuration,string(exception.message));
    end
    results=[results;row]; %#ok<AGROW>
    save(fullfile(dataFolder,'stage15_results.mat'),'results','configurations');
end
writetable(results,fullfile(dataFolder,'stage15_results.csv'));
plotStage15Results(results);
fprintf('\nSaved Stage 15 results to %s\n',dataFolder);
end

function row=failureRow15(c,message)
row=table(string(c.Name),c.PolicyMode,c.SafetyMode,c.ScenarioMode,c.FaultSchedule, ...
    false,nan,nan,nan,nan,nan,nan,nan,nan,nan,nan,nan,nan,nan,nan,false,message, ...
    'VariableNames',{'Name','PolicyMode','SafetyMode','ScenarioMode','FaultSchedule', ...
    'MissionSuccess','MinimumSeparation','TrackedMinimumSeparation','ViolationFraction','FinalGoalError', ...
    'PathLength','ControlEffort','CBFInterventionRate','MaximumSlack','FallbackCount', ...
    'StalePacketFraction','MeanGraphNodes','MeanGraphEdges','MeanTopGATScore', ...
    'MaximumMotorSpeed','RunSucceeded','ErrorMessage'});
end
