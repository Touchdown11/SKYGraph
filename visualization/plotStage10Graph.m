%% PLOTSTAGE10GRAPH Plot fixed graph and transparent threat baseline
if ~exist('out','var')
    error('Run first: out = sim("quadrotor_stage10_graph");');
end

[t,mask] = readStage10Log(out,'graph10_mask_log',17);
[~,nodeId] = readStage10Log(out,'graph10_id_log',17);
[~,nodeType] = readStage10Log(out,'graph10_type_log',17);
[~,adjacencyVector] = readStage10Log(out,'graph10_adjacency_log',289);
[~,edgeCount] = readStage10Log(out,'graph10_edge_count_log',1);
[~,threat] = readStage10Log(out,'graph10_threat_log',17);
[~,topId] = readStage10Log(out,'graph10_top_id_log',5);
[~,topScore] = readStage10Log(out,'graph10_top_score_log',5);
[tRelative,relativeVector] = readStage10Log(out,'track9_relative_log',48);

mask = mask > 0.5;

figure('Name','Stage 10 graph size');
subplot(2,1,1);
stairs(t,sum(mask,2),'LineWidth',1.4);
grid on;
ylabel('Active nodes');
title('Fixed capacity 17: ego plus up to 16 entities');
subplot(2,1,2);
stairs(t,edgeCount,'LineWidth',1.4);
grid on;
xlabel('Time (s)');
ylabel('Undirected edges');

figure('Name','Stage 10 threat scores');
maskedThreat = threat;
maskedThreat(~mask) = NaN;
imagesc(t,0:16,maskedThreat.');
axis xy;
xlabel('Time (s)');
ylabel('Graph node index');
yticks(0:16);
colorbar;
title('Transparent threat score, node 0 is ego');

figure('Name','Stage 10 top threats');
subplot(2,1,1);
stairs(t,topId,'LineWidth',1.1);
grid on;
ylabel('Entity ID');
legend('Rank 1','Rank 2','Rank 3','Rank 4','Rank 5','Location','eastoutside');
title('Top-five threat entity IDs');
subplot(2,1,2);
plot(t,topScore,'LineWidth',1.2);
grid on;
xlabel('Time (s)');
ylabel('Threat score');
legend('Rank 1','Rank 2','Rank 3','Rank 4','Rank 5','Location','eastoutside');

% Graph snapshot at the maximum rank-1 threat score.
[~,snapshotIndex] = max(topScore(:,1));
snapshotTime = t(snapshotIndex);
[~,relativeIndex] = min(abs(tRelative-snapshotTime));
relativePosition = reshape(relativeVector(relativeIndex,:).',3,16);
adjacency = reshape(adjacencyVector(snapshotIndex,:).',17,17) > 0.5;
nodePosition = zeros(2,17);
nodePosition(:,2:17) = relativePosition(1:2,:);

figure('Name','Stage 10 graph snapshot');
hold on;
grid on;
axis equal;
for firstNode = 1:17
    for secondNode = firstNode+1:17
        if adjacency(firstNode,secondNode)
            plot(nodePosition(1,[firstNode,secondNode]), ...
                nodePosition(2,[firstNode,secondNode]),'-', ...
                'Color',[0.65,0.70,0.78],'LineWidth',0.8);
        end
    end
end
colors = [0.2,0.8,0.3;0.15,0.68,1.0;0.58,0.42,0.25;1.0,0.3,0.2];
for node = 1:17
    if ~mask(snapshotIndex,node)
        continue;
    end
    if node == 1
        color = colors(1,:);
        label = 'EGO';
    else
        typeValue = round(nodeType(snapshotIndex,node));
        typeValue = min(max(typeValue,1),3);
        color = colors(typeValue+1,:);
        label = sprintf('%.0f',nodeId(snapshotIndex,node));
    end
    sizeValue = 50+250*threat(snapshotIndex,node);
    scatter(nodePosition(1,node),nodePosition(2,node),sizeValue,color,'filled');
    text(nodePosition(1,node),nodePosition(2,node)+0.08,label, ...
        'HorizontalAlignment','center');
end
xlabel('Relative X (m)');
ylabel('Relative Y (m)');
title(sprintf('Graph at %.2f s; marker size is threat score',snapshotTime));

function [time,data] = readStage10Log(simulationOutput,name,width)
names = simulationOutput.who;
if any(strcmp(names,name))
    signal = simulationOutput.get(name);
elseif evalin('base',['exist(''' name ''',''var'')'])
    signal = evalin('base',name);
else
    error('Missing log "%s".',name);
end
time = signal.Time(:);
data = squeeze(signal.Data);
if isvector(data)
    data = data(:);
elseif size(data,1) == numel(time)
    % Correct orientation.
elseif size(data,2) == numel(time)
    data = data.';
else
    error('%s has incompatible dimensions.',name);
end
if size(data,2) < width
    error('%s needs %d columns but has %d.',name,width,size(data,2));
end
data = data(:,1:width);
end
