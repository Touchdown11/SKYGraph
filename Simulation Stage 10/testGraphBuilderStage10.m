%% TESTGRAPHBUILDERSTAGE10 Validate graph dimensions and threat ranking
clear;
clc;
close all;

% Synthetic three-track test with a clearly dangerous unknown entity.
relativePosition = zeros(3,16);
relativeVelocity = zeros(3,16);
radius = zeros(16,1);
id = zeros(16,1);
type = zeros(16,1);
active = false(16,1);
confidence = zeros(16,1);
age = zeros(16,1);
sourceMask = zeros(16,1);
closing = zeros(16,1);
ttc = 1000*ones(16,1);

relativePosition(:,1) = [0.45;0;0];
relativeVelocity(:,1) = [-0.60;0;0];
radius(1)=0.10; id(1)=301; type(1)=3; active(1)=true;
confidence(1)=0.95; sourceMask(1)=4; closing(1)=0.60; ttc(1)=0.58;

relativePosition(:,2) = [2.50;0.20;0];
relativeVelocity(:,2) = [0;0;0];
radius(2)=0.35; id(2)=201; type(2)=2; active(2)=true;
confidence(2)=0.95; sourceMask(2)=2; closing(2)=0; ttc(2)=1000;

relativePosition(:,3) = [1.10;1.00;0.10];
relativeVelocity(:,3) = [0.05;-0.05;0];
radius(3)=0.16; id(3)=101; type(3)=1; active(3)=true;
confidence(3)=0.90; sourceMask(3)=1; closing(3)=0.02; ttc(3)=60;

[features,mask,nodeId,nodeType,adjacency,edgeCount,score,topNode,topId,topScore] = ...
    graphBuilderStage10(relativePosition,relativeVelocity,radius,id,type, ...
    active,confidence,age,sourceMask,closing,ttc); %#ok<ASGLU>

assert(isequal(size(features),[19,17]));
assert(isequal(size(mask),[17,1]));
assert(isequal(size(adjacency),[17,17]));
assert(mask(1) && sum(mask)==4);
assert(all(diag(adjacency)==0));
assert(isequal(adjacency,adjacency.'));
assert(edgeCount >= 3);
assert(topId(1)==301);
assert(topScore(1)>topScore(2));
assert(topNode(1)==1); % graph node index 1 after ego index 0
assert(nodeId(2)==301 && nodeType(2)==3);

% Empty graph must contain only ego.
emptyActive = false(16,1);
[~,emptyMask,~,~,emptyAdj,emptyEdges,~,~,emptyTopId] = ...
    graphBuilderStage10(zeros(3,16),zeros(3,16),zeros(16,1), ...
    zeros(16,1),zeros(16,1),emptyActive,zeros(16,1),zeros(16,1), ...
    zeros(16,1),zeros(16,1),1000*ones(16,1));
assert(sum(emptyMask)==1 && emptyEdges==0 && ~any(emptyAdj(:)));
assert(all(emptyTopId==0));

% Short full-pipeline integration using Stage 8 and Stage 9 functions.
clear realisticScenarioToFStage8 entityTrackerStage9
xEstimate = zeros(12,1);
xEstimate(3)=1.0;
time = (0:0.05:8).';
graphTime = zeros(ceil(length(time)/2),1);
nodeCountLog = zeros(size(graphTime));
edgeCountLog = zeros(size(graphTime));
topScoreLog = zeros(size(graphTime));
graphSample = 0;
for k = 1:length(time)
    [position,velocity,entityRadius,entityId,~,entityActive,telemetry,map, ...
        ground,ceiling,ambient,fault] = skyGraphScenarioStage8(time(k),5,0);
    [distance,valid,status,~,~,~,~,packetNew,packetAge] = ...
        realisticScenarioToFStage8(xEstimate,time(k),position,entityRadius, ...
        entityId,zeros(16,1),entityActive,ground,ceiling,ambient,fault);
    [~,~,trackRelative,trackRelVelocity,trackRadius,trackId,trackType, ...
        trackActive,trackConfidence,trackAge,trackSource,trackClosing,trackTTC] = ...
        entityTrackerStage9(xEstimate,distance,valid,status,packetNew,packetAge, ...
        position,velocity,entityRadius,entityId,entityActive,telemetry,map);
    if mod(k-1,2)==0
        graphSample=graphSample+1;
        [~,nodeMask,~,~,~,edges,~,~,~,rankedScore] = ...
            graphBuilderStage10(trackRelative,trackRelVelocity,trackRadius, ...
            trackId,trackType,trackActive,trackConfidence,trackAge, ...
            trackSource,trackClosing,trackTTC);
        graphTime(graphSample)=time(k);
        nodeCountLog(graphSample)=sum(nodeMask);
        edgeCountLog(graphSample)=edges;
        topScoreLog(graphSample)=rankedScore(1);
    end
end
graphTime=graphTime(1:graphSample);
nodeCountLog=nodeCountLog(1:graphSample);
edgeCountLog=edgeCountLog(1:graphSample);
topScoreLog=topScoreLog(1:graphSample);
assert(all(nodeCountLog>=8));
assert(all(edgeCountLog>0));
assert(all(topScoreLog>=0 & topScoreLog<=1));

fprintf('\nSTAGE 10 GRAPH VALIDATION\n');
fprintf('=========================\n');
fprintf('Feature matrix 19x17: PASS\n');
fprintf('Symmetric fixed adjacency: PASS\n');
fprintf('Empty graph behavior: PASS\n');
fprintf('Dangerous unknown ranked first: PASS (score %.3f)\n',topScore(1));
fprintf('Integrated node-count range: %d to %d\n', ...
    min(nodeCountLog),max(nodeCountLog));
fprintf('Integrated edge-count range: %d to %d\n', ...
    min(edgeCountLog),max(edgeCountLog));
fprintf('RESULT: PASS - Stage 10 graph and baseline are ready.\n\n');

figure('Name','Stage 10 integration graph statistics');
subplot(3,1,1);
stairs(graphTime,nodeCountLog,'LineWidth',1.3);
grid on;
ylabel('Active nodes');
subplot(3,1,2);
stairs(graphTime,edgeCountLog,'LineWidth',1.3);
grid on;
ylabel('Edges');
subplot(3,1,3);
plot(graphTime,topScoreLog,'LineWidth',1.3);
grid on;
xlabel('Time (s)');
ylabel('Top threat score');
