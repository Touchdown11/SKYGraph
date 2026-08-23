function [nodeFeatures,nodeMask,nodeId,nodeType,adjacency,edgeCount, ...
    threatScore,topNodeIndex,topEntityId,topScore] = ...
    graphBuilderStage10(relativePosition,relativeVelocity,trackRadius, ...
    trackId,trackType,trackActive,confidence,age,sourceMask, ...
    closingSpeed,timeToCollision)
%GRAPHBUILDERSTAGE10 Fixed graph and transparent threat-ranking baseline.
%
% Node 1 is EGO. Nodes 2:17 correspond to track slots 1:16.
% Feature dimension is 19. Graph capacity is always 17 nodes.

%#codegen

featureCount = 19;
nodeCapacity = 17;
topK = 5;

nodeFeatures = zeros(featureCount,nodeCapacity);
nodeMask = false(nodeCapacity,1);
nodeId = zeros(nodeCapacity,1);
nodeType = zeros(nodeCapacity,1);
adjacency = false(nodeCapacity,nodeCapacity);
threatScore = zeros(nodeCapacity,1);

% Ego node: type one-hot [ego cooperative building unknown].
nodeMask(1) = true;
nodeFeatures(1,1) = 1.0;
nodeId(1) = 0.0;
nodeType(1) = 0.0;

for track = 1:16
    node = track+1;
    if ~trackActive(track)
        continue;
    end
    nodeMask(node) = true;
    nodeId(node) = trackId(track);
    nodeType(node) = trackType(track);

    % Type one-hot.
    if trackType(track) == 1
        nodeFeatures(2,node) = 1.0;
    elseif trackType(track) == 2
        nodeFeatures(3,node) = 1.0;
    else
        nodeFeatures(4,node) = 1.0;
    end

    % Normalized kinematic and source features.
    nodeFeatures(5:7,node) = relativePosition(:,track)/3.0;
    nodeFeatures(8:10,node) = relativeVelocity(:,track)/2.0;
    centreDistance = norm(relativePosition(:,track));
    surfaceDistance = max(centreDistance-trackRadius(track),0.0);
    nodeFeatures(11,node) = min(surfaceDistance/3.0,1.0);
    nodeFeatures(12,node) = min(max(closingSpeed(track)/2.0,-1.0),1.0);
    nodeFeatures(13,node) = min(timeToCollision(track)/10.0,1.0);
    nodeFeatures(14,node) = min(trackRadius(track),1.0);
    nodeFeatures(15,node) = min(max(confidence(track),0.0),1.0);
    nodeFeatures(16,node) = min(max(age(track)/0.50,0.0),1.0);
    nodeFeatures(17,node) = double(sourceMaskHasStage10(sourceMask(track),1.0));
    nodeFeatures(18,node) = double(sourceMaskHasStage10(sourceMask(track),2.0));
    nodeFeatures(19,node) = double(sourceMaskHasStage10(sourceMask(track),4.0));

    % Explainable non-neural threat baseline.
    proximityTerm = exp(-surfaceDistance/0.80);
    closingTerm = min(max(closingSpeed(track)/1.0,0.0),1.0);
    if closingSpeed(track) > 0.01
        ttcTerm = 1.0-min(timeToCollision(track)/8.0,1.0);
    else
        ttcTerm = 0.0;
    end
    uncertaintyTerm = min(max((1.0-confidence(track))+age(track)/0.50,0.0),1.0);
    if trackType(track) == 3
        typeWeight = 1.0;
    elseif trackType(track) == 1
        typeWeight = 0.85;
    else
        typeWeight = 0.75;
    end
    score = typeWeight*confidence(track)*( ...
        0.45*proximityTerm+0.30*ttcTerm+0.20*closingTerm) ...
        +0.05*uncertaintyTerm;
    threatScore(node) = min(max(score,0.0),1.0);
end

% Symmetric radius graph. Ego connects to all active tracks within 3 m;
% track-track edges use 1.5 m relative separation.
for node = 2:nodeCapacity
    if nodeMask(node)
        track = node-1;
        if norm(relativePosition(:,track)) <= 3.0
            adjacency(1,node) = true;
            adjacency(node,1) = true;
        end
    end
end
for firstNode = 2:nodeCapacity
    if ~nodeMask(firstNode)
        continue;
    end
    firstTrack = firstNode-1;
    for secondNode = firstNode+1:nodeCapacity
        if ~nodeMask(secondNode)
            continue;
        end
        secondTrack = secondNode-1;
        if norm(relativePosition(:,firstTrack)-relativePosition(:,secondTrack)) <= 1.5
            adjacency(firstNode,secondNode) = true;
            adjacency(secondNode,firstNode) = true;
        end
    end
end
edgeCount = sum(sum(adjacency))/2.0;

% Fixed-size top-k selection without variable-size sorting.
topNodeIndex = zeros(topK,1);
topEntityId = zeros(topK,1);
topScore = zeros(topK,1);
used = false(nodeCapacity,1);
used(1) = true; % ego cannot be a threat candidate
for rank = 1:topK
    bestScore = -1.0;
    bestNode = 0;
    for node = 2:nodeCapacity
        if nodeMask(node) && ~used(node) && threatScore(node) > bestScore
            bestScore = threatScore(node);
            bestNode = node;
        end
    end
    if bestNode > 0
        used(bestNode) = true;
        topNodeIndex(rank) = bestNode-1; % zero-based graph index, ego is 0
        topEntityId(rank) = nodeId(bestNode);
        topScore(rank) = threatScore(bestNode);
    end
end
end

function present = sourceMaskHasStage10(maskValue,bitValue)
% Source masks are small integer-valued doubles.
integerMask = round(maskValue);
integerBit = round(bitValue);
present = mod(floor(integerMask/integerBit),2) == 1;
end
