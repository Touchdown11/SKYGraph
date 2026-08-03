%% PLOTSTAGE13PPO Plot PPO actions and CBF-filtered commands
if ~exist('out','var'),error('Run quadrotor_stage13_ppo first.');end
[tAction,action]=readStage13Log(out,'ppo13_action_log',3);
[tSelected,selected]=readStage13Log(out,'ppo13_selected_log',3);
[tSource,source]=readStage13Log(out,'ppo13_source_log',1);
[tCBF,safe]=readStage13Log(out,'cbf11_safe_log',3);
[tIntervention,intervention]=readStage13Log(out,'cbf11_intervention_log',1);
[tSlack,slack]=readStage13Log(out,'cbf11_slack_log',1);

figure('Name','Stage 13 PPO normalized action');
plot(tAction,action,'LineWidth',1.2);grid on;xlabel('Time (s)');
ylabel('Normalized action');ylim([-1.1,1.1]);
legend('X','Y','Z','Location','best');

figure('Name','Stage 13 nominal and CBF-safe command');
for axis=1:3
    subplot(3,1,axis);plot(tSelected,selected(:,axis),'--','LineWidth',1.1);hold on;
    plot(tCBF,safe(:,axis),'LineWidth',1.3);grid on;
    ylabel(sprintf('a_%d',axis));
    if axis==1,legend('Selected nominal','CBF safe','Location','best');end
end
xlabel('Time (s)');

figure('Name','Stage 13 policy and safety activity');
subplot(3,1,1);stairs(tSource,source,'LineWidth',1.3);grid on;
ylabel('Policy');yticks([0,1]);yticklabels({'Waypoint','PPO'});
subplot(3,1,2);stairs(tIntervention,intervention,'LineWidth',1.3);grid on;
ylabel('CBF active');
subplot(3,1,3);plot(tSlack,slack,'LineWidth',1.3);grid on;
xlabel('Time (s)');ylabel('Slack');

fprintf('\nSTAGE 13 SIMULINK SUMMARY\n');
fprintf('Maximum normalized action magnitude: %.3f\n',max(abs(action),[],'all'));
fprintf('PPO selected for %.1f %% of samples\n',100*mean(source>0.5));
fprintf('CBF intervention rate: %.1f %%\n',100*mean(intervention>0.5));
fprintf('Maximum CBF slack: %.4f\n\n',max(slack));

function [time,data]=readStage13Log(simulationOutput,name,width)
names=simulationOutput.who;
if any(strcmp(names,name)),signal=simulationOutput.get(name);
elseif evalin('base',['exist(''' name ''',''var'')']),signal=evalin('base',name);
else,error('Missing log "%s".',name);end
time=signal.Time(:);data=squeeze(signal.Data);
if isvector(data),data=data(:);
elseif size(data,1)==numel(time)
elseif size(data,2)==numel(time),data=data.';
else,error('%s has incompatible dimensions.',name);end
if size(data,2)<width,error('%s needs %d columns.',name,width);end
data=data(:,1:width);
end
