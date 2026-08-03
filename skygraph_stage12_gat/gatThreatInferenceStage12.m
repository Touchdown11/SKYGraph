function [threatScore,attentionVector,topEntityId,topScore] = ...
    gatThreatInferenceStage12(featureVector,nodeMask,adjacencyVector,nodeId)
%GATTHREATINFERENCESTAGE12 Fixed-size one-head GAT inference.
% Loads trained numeric weights from gatThreatWeightsStage12.mat.

%#codegen

persistent W aSource aTarget wOutput bOutput loaded
if isempty(loaded)
    data=coder.load('gatThreatWeightsStage12.mat', ...
        'W','aSource','aTarget','wOutput','bOutput');
    W=data.W; aSource=data.aSource; aTarget=data.aTarget;
    wOutput=data.wOutput; bOutput=data.bOutput;
    loaded=true;
end

features=reshape(featureVector,19,17);
adjacency=reshape(adjacencyVector,17,17);
hidden=W*features;
attention=zeros(17,17);
threatScore=zeros(17,1);

for target=1:17
    if ~nodeMask(target)
        continue;
    end
    logits=-1.0e6*ones(17,1);
    maximumLogit=-1.0e6;
    for source=1:17
        connected=(target==source) || adjacency(target,source);
        if nodeMask(source) && connected
            value=dot(aSource,hidden(:,target))+dot(aTarget,hidden(:,source));
            if value<0
                value=0.2*value;
            end
            logits(source)=value;
            maximumLogit=max(maximumLogit,value);
        end
    end
    denominator=0.0;
    for source=1:17
        if logits(source)>-1.0e5
            denominator=denominator+exp(logits(source)-maximumLogit);
        end
    end
    aggregate=zeros(12,1);
    if denominator>0
        for source=1:17
            if logits(source)>-1.0e5
                weight=exp(logits(source)-maximumLogit)/denominator;
                attention(target,source)=weight;
                aggregate=aggregate+weight*hidden(:,source);
            end
        end
    end
    outputHidden=tanh(aggregate+hidden(:,target));
    logit=dot(wOutput,outputHidden)+bOutput;
    threatScore(target)=1.0/(1.0+exp(-logit));
end
threatScore(1)=0.0;

% Fixed top five entity IDs.
topEntityId=zeros(5,1);
topScore=zeros(5,1);
used=false(17,1); used(1)=true;
for rank=1:5
    best=-1.0; bestNode=0;
    for node=2:17
        if nodeMask(node) && ~used(node) && threatScore(node)>best
            best=threatScore(node); bestNode=node;
        end
    end
    if bestNode>0
        used(bestNode)=true;
        topEntityId(rank)=nodeId(bestNode);
        topScore(rank)=threatScore(bestNode);
    end
end
attentionVector=reshape(attention,289,1);
end
