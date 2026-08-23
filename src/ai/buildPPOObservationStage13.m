function observation = buildPPOObservationStage13(info)
%BUILDPPOOBSERVATIONSTAGE13 Build 42-element observation for training.

relativePosition=zeros(3,16);relativeVelocity=zeros(3,16);
radius=zeros(16,1);id=zeros(16,1);type=zeros(16,1);
active=false(16,1);confidence=zeros(16,1);age=zeros(16,1);
sourceMask=zeros(16,1);closing=zeros(16,1);ttc=1000*ones(16,1);
for k=1:8
    if ~info.ObstacleActive(k),continue;end
    r=info.ObstaclePosition(:,k)-info.Position;
    w=info.ObstacleVelocity(:,k)-info.Velocity;
    relativePosition(:,k)=r;relativeVelocity(:,k)=w;
    radius(k)=info.ObstacleRadius(k);id(k)=500+k;type(k)=info.ObstacleType(k);
    active(k)=true;confidence(k)=0.95;
    if type(k)==1,sourceMask(k)=1;elseif type(k)==2,sourceMask(k)=2;else,sourceMask(k)=4;end
    d=max(norm(r),1e-6);closing(k)=-dot(r,w)/d;
    if closing(k)>0.01,ttc(k)=max(d-radius(k),0)/closing(k);end
end
[features,mask,nodeId,~,adjacency,~,~,~,~,~]=graphBuilderStage10( ...
    relativePosition,relativeVelocity,radius,id,type,active,confidence,age, ...
    sourceMask,closing,ttc);
[gatScore,~,topId,topScore]=gatThreatInferenceStage12( ...
    reshape(features,323,1),mask,reshape(adjacency,289,1),nodeId);

observation=zeros(42,1);
observation(1:3)=(info.Goal-info.Position)/3.0;
observation(4:6)=info.Velocity/2.0;
observation(7:9)=info.PreviousSafeAcceleration./[3;3;5];
writeIndex=10;
for rank=1:3
    entityId=topId(rank);
    trackIndex=0;
    for k=1:8
        if active(k) && id(k)==entityId,trackIndex=k;break;end
    end
    if trackIndex>0
        observation(writeIndex:writeIndex+2)=relativePosition(:,trackIndex)/3.0;
        observation(writeIndex+3:writeIndex+5)=relativeVelocity(:,trackIndex)/2.0;
        observation(writeIndex+6)=radius(trackIndex);
        observation(writeIndex+7)=topScore(rank);
        obstacleType=round(type(trackIndex));
        if obstacleType>=1 && obstacleType<=3
            observation(writeIndex+7+obstacleType)=1.0;
        end
    end
    writeIndex=writeIndex+11;
end
% Keep finite and bounded for network training.
observation=min(max(observation,-5.0),5.0);
% Reference gatScore so static analysis keeps the same complete inference path.
if any(~isfinite(gatScore)),observation=zeros(42,1);end
end
