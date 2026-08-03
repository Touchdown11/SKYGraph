function report = evaluatePPOStage13(episodeCount)
%EVALUATEPPOSTAGE13 Evaluate trained greedy PPO with CBF active.

if nargin<1,episodeCount=50;end
file=fullfile(fileparts(mfilename('fullpath')),'ppoAgentStage13.mat');
if ~isfile(file),error('Run trainPPOStage13 first.');end
loaded=load(file,'agent');agent=loaded.agent;
agent.UseExplorationPolicy=false;
[environment,~,~]=createSkyGraphPPOEnvironmentStage13();
success=false(episodeCount,1);collision=false(episodeCount,1);
minimumSeparation=zeros(episodeCount,1);interventionRate=zeros(episodeCount,1);
episodeReward=zeros(episodeCount,1);

for episode=1:episodeCount
    observation=reset(environment);
    done=false;steps=0;rewardSum=0;lastInfo=[];
    while ~done && steps<200
        action=getAction(agent,observation);
        if iscell(action),action=action{1};end
        [observation,reward,done,info]=step(environment,action);
        rewardSum=rewardSum+reward;steps=steps+1;lastInfo=info;
    end
    success(episode)=lastInfo.Success;
    collision(episode)=lastInfo.Collision;
    minimumSeparation(episode)=lastInfo.MinimumSeparation;
    interventionRate(episode)=lastInfo.InterventionCount/max(lastInfo.StepCount,1);
    episodeReward(episode)=rewardSum;
end

report=struct;
report.SuccessRate=mean(success);
report.CollisionRate=mean(collision);
report.MeanMinimumSeparation=mean(minimumSeparation);
report.MeanInterventionRate=mean(interventionRate);
report.MeanReward=mean(episodeReward);
fprintf('\nSTAGE 13 PPO EVALUATION\n');
fprintf('Success rate: %.1f %%\n',100*report.SuccessRate);
fprintf('Collision rate: %.1f %%\n',100*report.CollisionRate);
fprintf('Mean minimum separation: %.3f m\n',report.MeanMinimumSeparation);
fprintf('Mean CBF intervention rate: %.1f %%\n',100*report.MeanInterventionRate);
fprintf('Mean reward: %.2f\n\n',report.MeanReward);

figure('Name','Stage 13 PPO evaluation');
subplot(3,1,1);bar([mean(success),mean(collision)]);ylim([0,1]);
xticklabels({'Success','Collision'});ylabel('Rate');grid on;
subplot(3,1,2);histogram(minimumSeparation);xlabel('Minimum separation (m)');grid on;
subplot(3,1,3);histogram(interventionRate);xlabel('CBF intervention fraction');grid on;
end
