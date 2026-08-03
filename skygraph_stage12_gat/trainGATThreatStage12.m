function report = trainGATThreatStage12(sampleCount,epochCount)
%TRAINGATTHREATSTAGE12 Train a one-head fixed-size GAT threat encoder.

if nargin<1, sampleCount=1200; end
if nargin<2, epochCount=30; end
rng(12012,'twister');
dataset=generateGATDatasetStage12(sampleCount,12012);
order=randperm(sampleCount);
trainCount=floor(0.80*sampleCount);
trainIndex=order(1:trainCount);
validationIndex=order(trainCount+1:end);

hiddenSize=12;
W=dlarray(single(randn(hiddenSize,19)*sqrt(2/19)));
aSource=dlarray(single(0.10*randn(hiddenSize,1)));
aTarget=dlarray(single(0.10*randn(hiddenSize,1)));
wOutput=dlarray(single(0.10*randn(hiddenSize,1)));
bOutput=dlarray(single(0));

avgW=[];avgSqW=[];avgAS=[];avgSqAS=[];avgAT=[];avgSqAT=[];
avgWO=[];avgSqWO=[];avgBO=[];avgSqBO=[];
learningRate=0.003;
batchSize=32;
iteration=0;
lossHistory=zeros(epochCount,1);

for epoch=1:epochCount
    shuffled=trainIndex(randperm(numel(trainIndex)));
    epochLoss=0;batchCounter=0;
    for startIndex=1:batchSize:numel(shuffled)
        batch=shuffled(startIndex:min(startIndex+batchSize-1,numel(shuffled)));
        X=dlarray(dataset.Features(:,:,batch));
        Y=dlarray(dataset.Labels(:,batch));
        lossMask=single(dataset.TrainingMask(:,batch));
        connection=dataset.Adjacency(:,:,batch);
        for b=1:numel(batch)
            active=dataset.NodeMask(:,batch(b));
            connection(:,:,b)=connection(:,:,b) | diag(active);
        end
        [loss,gW,gAS,gAT,gWO,gBO]=dlfeval(@gatLossStage12, ...
            W,aSource,aTarget,wOutput,bOutput,X,Y,lossMask,connection);
        iteration=iteration+1;
        [W,avgW,avgSqW]=adamupdate(W,gW,avgW,avgSqW,iteration,learningRate);
        [aSource,avgAS,avgSqAS]=adamupdate(aSource,gAS,avgAS,avgSqAS,iteration,learningRate);
        [aTarget,avgAT,avgSqAT]=adamupdate(aTarget,gAT,avgAT,avgSqAT,iteration,learningRate);
        [wOutput,avgWO,avgSqWO]=adamupdate(wOutput,gWO,avgWO,avgSqWO,iteration,learningRate);
        [bOutput,avgBO,avgSqBO]=adamupdate(bOutput,gBO,avgBO,avgSqBO,iteration,learningRate);
        epochLoss=epochLoss+double(extractdata(loss));
        batchCounter=batchCounter+1;
    end
    lossHistory(epoch)=epochLoss/max(batchCounter,1);
    fprintf('Epoch %2d/%2d  loss %.6f\n',epoch,epochCount,lossHistory(epoch));
end

W=double(extractdata(W));aSource=double(extractdata(aSource));
aTarget=double(extractdata(aTarget));wOutput=double(extractdata(wOutput));
bOutput=double(extractdata(bOutput));
outputFile=fullfile(fileparts(mfilename('fullpath')),'gatThreatWeightsStage12.mat');
save(outputFile,'W','aSource','aTarget','wOutput','bOutput');
clear gatThreatInferenceStage12

% Held-out evaluation.
gatSquaredError=0;baselineSquaredError=0;labelCount=0;
gatTop1=0;baselineTop1=0;gatTop3=0;
for index=validationIndex
    features=double(dataset.Features(:,:,index));
    mask=dataset.NodeMask(:,index);
    adjacency=dataset.Adjacency(:,:,index);
    [prediction,~]=gatThreatForwardNumericStage12(W,aSource,aTarget, ...
        wOutput,bOutput,features,mask,adjacency);
    label=double(dataset.Labels(:,index));
    baseline=double(dataset.Baseline(:,index));
    evaluationMask=dataset.TrainingMask(:,index);
    gatSquaredError=gatSquaredError+sum((prediction(evaluationMask)-label(evaluationMask)).^2);
    baselineSquaredError=baselineSquaredError+sum((baseline(evaluationMask)-label(evaluationMask)).^2);
    labelCount=labelCount+sum(evaluationMask);
    activeNodes=find(evaluationMask);
    [~,truthLocal]=max(label(activeNodes)); truthNode=activeNodes(truthLocal);
    [~,gatOrder]=sort(prediction(activeNodes),'descend');
    [~,baseOrder]=sort(baseline(activeNodes),'descend');
    gatTop1=gatTop1+(activeNodes(gatOrder(1))==truthNode);
    baselineTop1=baselineTop1+(activeNodes(baseOrder(1))==truthNode);
    topCount=min(3,numel(gatOrder));
    gatTop3=gatTop3+any(activeNodes(gatOrder(1:topCount))==truthNode);
end
validationCount=numel(validationIndex);
report=struct;
report.WeightFile=outputFile;
report.FinalTrainingLoss=lossHistory(end);
report.GATMSE=gatSquaredError/max(labelCount,1);
report.BaselineMSE=baselineSquaredError/max(labelCount,1);
report.GATTop1Accuracy=gatTop1/validationCount;
report.BaselineTop1Accuracy=baselineTop1/validationCount;
report.GATTop3Recall=gatTop3/validationCount;

fprintf('\nSTAGE 12 GAT EVALUATION\n');
fprintf('GAT MSE: %.5f\n',report.GATMSE);
fprintf('Baseline MSE: %.5f\n',report.BaselineMSE);
fprintf('GAT top-1 accuracy: %.2f %%\n',100*report.GATTop1Accuracy);
fprintf('Baseline top-1 accuracy: %.2f %%\n',100*report.BaselineTop1Accuracy);
fprintf('GAT top-3 recall: %.2f %%\n',100*report.GATTop3Recall);
fprintf('Saved: %s\n\n',outputFile);

figure('Name','Stage 12 GAT training');
plot(1:epochCount,lossHistory,'LineWidth',1.5);grid on;
xlabel('Epoch');ylabel('Training MSE');title('One-head GAT threat encoder');
end

function [loss,gW,gAS,gAT,gWO,gBO]=gatLossStage12( ...
    W,aSource,aTarget,wOutput,bOutput,X,Y,lossMask,connection)
[prediction,~]=gatForwardDLStage12(W,aSource,aTarget,wOutput,bOutput,X,connection);
maskDL=dlarray(lossMask);
difference=(prediction-Y).*maskDL;
loss=sum(difference.^2,'all')/max(sum(maskDL,'all'),1);
[gW,gAS,gAT,gWO,gBO]=dlgradient(loss,W,aSource,aTarget,wOutput,bOutput);
end

function [prediction,attention]=gatForwardDLStage12( ...
    W,aSource,aTarget,wOutput,bOutput,X,connection)
batchCount=size(X,3);
hidden=pagemtimes(W,X); % 12x17xB
sourceScore=pagemtimes(reshape(aSource,1,12),hidden); % 1x17xB
targetScore=pagemtimes(reshape(aTarget,1,12),hidden);
energy=permute(sourceScore,[2,1,3])+targetScore; % target x source x B
energy=max(energy,0)+0.2*min(energy,0);
maskPenalty=dlarray(single(~connection)*(-1.0e4));
energy=energy+maskPenalty;
energyMaximum=max(energy,[],2);
exponential=exp(energy-energyMaximum);
attention=exponential./sum(exponential,2);
aggregate=pagemtimes(hidden,permute(attention,[2,1,3]));
outputHidden=tanh(aggregate+hidden);
logits=pagemtimes(reshape(wOutput,1,12),outputHidden)+bOutput;
prediction=reshape(1./(1+exp(-logits)),17,batchCount);
end
