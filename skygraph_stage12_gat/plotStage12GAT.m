%% PLOTSTAGE12GAT Compare learned GAT with transparent Stage 10 baseline
if ~exist('out','var')
    error('Run first: out = sim("quadrotor_stage12_gat");');
end

[t,gatThreat]=readStage12Log(out,'gat12_threat_log',17);
[~,attentionVector]=readStage12Log(out,'gat12_attention_log',289);
[~,gatTopId]=readStage12Log(out,'gat12_top_id_log',5);
[~,gatTopScore]=readStage12Log(out,'gat12_top_score_log',5);
[~,baselineThreat]=readStage12Log(out,'graph10_threat_log',17);
[~,baselineTopId]=readStage12Log(out,'graph10_top_id_log',5);
[~,mask]=readStage12Log(out,'graph10_mask_log',17);
[~,nodeId]=readStage12Log(out,'graph10_id_log',17);
[~,nodeType]=readStage12Log(out,'graph10_type_log',17);
mask=mask>0.5;

figure('Name','Stage 12 GAT threat scores');
maskedGAT=gatThreat;maskedGAT(~mask)=NaN;
imagesc(t,0:16,maskedGAT.');axis xy;colorbar;
xlabel('Time (s)');ylabel('Graph node index');yticks(0:16);
title('Learned GAT threat score; node 0 is ego');

figure('Name','Stage 12 learned versus transparent ranking');
subplot(2,1,1);
plot(t,gatTopScore(:,1),'LineWidth',1.4);hold on;
plot(t,max(baselineThreat,[],2),'--','LineWidth',1.2);grid on;
ylabel('Rank-1 score');legend('GAT','Transparent baseline','Location','best');
subplot(2,1,2);
stairs(t,gatTopId(:,1),'LineWidth',1.2);hold on;
stairs(t,baselineTopId(:,1),'--','LineWidth',1.2);grid on;
xlabel('Time (s)');ylabel('Rank-1 entity ID');
legend('GAT','Transparent baseline','Location','best');

figure('Name','Stage 12 top five GAT entities');
subplot(2,1,1);stairs(t,gatTopId,'LineWidth',1.0);grid on;
ylabel('Entity ID');legend('1','2','3','4','5','Location','eastoutside');
subplot(2,1,2);plot(t,gatTopScore,'LineWidth',1.0);grid on;
xlabel('Time (s)');ylabel('GAT score');legend('1','2','3','4','5','Location','eastoutside');

% Attention snapshot when rank-1 GAT score is largest.
[~,snapshot]=max(gatTopScore(:,1));
attention=reshape(attentionVector(snapshot,:).',17,17);
figure('Name','Stage 12 attention snapshot');
imagesc(0:16,0:16,attention);axis xy;colorbar;
xlabel('Source/neighbor graph node');ylabel('Target graph node');
title(sprintf('GAT attention at %.2f s',t(snapshot)));

agreement=mean(gatTopId(:,1)==baselineTopId(:,1));
fprintf('\nSTAGE 12 RUN SUMMARY\n');
fprintf('GAT/baseline rank-1 ID agreement: %.2f %%\n',100*agreement);
fprintf('Maximum GAT score: %.3f\n',max(gatTopScore(:,1)));
fprintf('Mean active nodes: %.2f\n',mean(sum(mask,2)));
activeTypes=nodeType(snapshot,mask(snapshot,:));
activeIds=nodeId(snapshot,mask(snapshot,:));
fprintf('Snapshot active IDs: ');fprintf('%.0f ',activeIds);fprintf('\n');
fprintf('Snapshot active types: ');fprintf('%.0f ',activeTypes);fprintf('\n\n');

function [time,data]=readStage12Log(simulationOutput,name,width)
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
