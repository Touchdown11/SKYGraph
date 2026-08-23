function [score,attention] = gatThreatForwardNumericStage12( ...
    W,aSource,aTarget,wOutput,bOutput,features,nodeMask,adjacency)
%GATTHREATFORWARDNUMERICSTAGE12 Numeric forward pass for evaluation.

hidden=W*features;
attention=zeros(17,17);
score=zeros(17,1);
for target=1:17
    if ~nodeMask(target), continue; end
    logits=-1e6*ones(17,1); maximum=-1e6;
    for source=1:17
        if nodeMask(source) && (target==source || adjacency(target,source))
            value=aSource.'*hidden(:,target)+aTarget.'*hidden(:,source);
            if value<0, value=0.2*value; end
            logits(source)=value; maximum=max(maximum,value);
        end
    end
    denominator=0;
    for source=1:17
        if logits(source)>-1e5
            denominator=denominator+exp(logits(source)-maximum);
        end
    end
    aggregate=zeros(12,1);
    for source=1:17
        if logits(source)>-1e5
            weight=exp(logits(source)-maximum)/denominator;
            attention(target,source)=weight;
            aggregate=aggregate+weight*hidden(:,source);
        end
    end
    outputHidden=tanh(aggregate+hidden(:,target));
    score(target)=1/(1+exp(-(wOutput.'*outputHidden+bOutput)));
end
score(1)=0;
end
