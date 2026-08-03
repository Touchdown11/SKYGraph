%% TESTGATSTAGE12 Train if needed, then validate fixed inference
clear;
clc;
close all;

weightFile=fullfile(fileparts(mfilename('fullpath')),'gatThreatWeightsStage12.mat');
if ~isfile(weightFile)
    fprintf('No trained weight file found. Starting compact training run...\n');
    trainingReport=trainGATThreatStage12(800,20); %#ok<NASGU>
end
addpath(fileparts(mfilename('fullpath')),'-begin');
clear gatThreatInferenceStage12

dataset=generateGATDatasetStage12(200,9912);
nodeId=(0:16).';
gatSquaredError=0;baselineSquaredError=0;labelCount=0;
top1Correct=0;top3Correct=0;

for sample=1:200
    featureVector=reshape(double(dataset.Features(:,:,sample)),323,1);
    mask=dataset.NodeMask(:,sample);
    adjacencyVector=reshape(dataset.Adjacency(:,:,sample),289,1);
    [score,attentionVector,topId,topScore]=gatThreatInferenceStage12( ...
        featureVector,mask,adjacencyVector,nodeId);
    assert(isequal(size(score),[17,1]));
    assert(isequal(size(attentionVector),[289,1]));
    assert(isequal(size(topId),[5,1]));
    assert(all(isfinite(score)) && all(score>=0) && all(score<=1));
    assert(all(diff(topScore)<=1e-12));

    attention=reshape(attentionVector,17,17);
    for node=1:17
        if mask(node)
            assert(abs(sum(attention(node,:))-1)<1e-6);
        else
            assert(sum(attention(node,:))==0);
        end
    end

    evaluationMask=dataset.TrainingMask(:,sample);
    label=double(dataset.Labels(:,sample));
    baseline=double(dataset.Baseline(:,sample));
    gatSquaredError=gatSquaredError+sum((score(evaluationMask)-label(evaluationMask)).^2);
    baselineSquaredError=baselineSquaredError+sum((baseline(evaluationMask)-label(evaluationMask)).^2);
    labelCount=labelCount+sum(evaluationMask);
    activeNodes=find(evaluationMask);
    [~,truthLocal]=max(label(activeNodes));truthNode=activeNodes(truthLocal);
    [~,predictionOrder]=sort(score(activeNodes),'descend');
    top1Correct=top1Correct+(activeNodes(predictionOrder(1))==truthNode);
    topCount=min(3,numel(predictionOrder));
    top3Correct=top3Correct+any(activeNodes(predictionOrder(1:topCount))==truthNode);
end

gatMSE=gatSquaredError/labelCount;
baselineMSE=baselineSquaredError/labelCount;
fprintf('\nSTAGE 12 GAT VALIDATION\n');
fprintf('=======================\n');
fprintf('GAT MSE: %.5f\n',gatMSE);
fprintf('Transparent baseline MSE: %.5f\n',baselineMSE);
fprintf('GAT top-1 accuracy: %.2f %%\n',100*top1Correct/200);
fprintf('GAT top-3 recall: %.2f %%\n',100*top3Correct/200);
fprintf('Attention row normalization: PASS\n');
fprintf('Fixed-size numeric inference: PASS\n');
fprintf('RESULT: PASS - Stage 12 GAT inference is ready for Simulink.\n\n');
