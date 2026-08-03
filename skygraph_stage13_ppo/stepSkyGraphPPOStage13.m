function [nextObservation,reward,isDone,info] = ...
    stepSkyGraphPPOStage13(action,info)
%STEPSKYGRAPHPPOSTAGE13 Double-integrator training with CBF filtering.

if iscell(action),action=action{1};end
action=min(max(double(action(:)),-1.0),1.0);
nominalAcceleration=action.*[2.0;2.0;1.5];

relativePosition=zeros(3,16);relativeVelocity=zeros(3,16);
radius=zeros(16,1);active=false(16,1);confidence=zeros(16,1);age=zeros(16,1);
for k=1:8
    if info.ObstacleActive(k)
        relativePosition(:,k)=info.ObstaclePosition(:,k)-info.Position;
        relativeVelocity(:,k)=info.ObstacleVelocity(:,k)-info.Velocity;
        radius(k)=info.ObstacleRadius(k);active(k)=true;confidence(k)=0.95;
    end
end
[safeAcceleration,intervention,slack,~,~,~,~,~,~]= ...
    cbfSafetyShieldStage11(nominalAcceleration,info.Velocity, ...
    relativePosition,relativeVelocity,radius,active,confidence,age);

Ts=0.10;
oldGoalDistance=norm(info.Goal-info.Position);
info.Velocity=info.Velocity+Ts*safeAcceleration;
info.Velocity(1:2)=min(max(info.Velocity(1:2),-2.0),2.0);
info.Velocity(3)=min(max(info.Velocity(3),-1.0),1.0);
info.Position=info.Position+Ts*info.Velocity;

% Move non-building obstacles and bounce at the training volume boundary.
for k=1:8
    if ~info.ObstacleActive(k),continue;end
    info.ObstaclePosition(:,k)=info.ObstaclePosition(:,k)+Ts*info.ObstacleVelocity(:,k);
    for axis=1:2
        if abs(info.ObstaclePosition(axis,k))>2.8
            info.ObstaclePosition(axis,k)=sign(info.ObstaclePosition(axis,k))*2.8;
            info.ObstacleVelocity(axis,k)=-info.ObstacleVelocity(axis,k);
        end
    end
    if info.ObstaclePosition(3,k)<0.20 || info.ObstaclePosition(3,k)>2.20
        info.ObstaclePosition(3,k)=min(max(info.ObstaclePosition(3,k),0.20),2.20);
        info.ObstacleVelocity(3,k)=-info.ObstacleVelocity(3,k);
    end
end

minimumSeparation=100;
for k=1:8
    if info.ObstacleActive(k)
        clearance=norm(info.ObstaclePosition(:,k)-info.Position) ...
            -(0.22+info.ObstacleRadius(k));
        minimumSeparation=min(minimumSeparation,clearance);
    end
end
info.MinimumSeparation=min(info.MinimumSeparation,minimumSeparation);
info.StepCount=info.StepCount+1;
info.PreviousSafeAcceleration=safeAcceleration;
info.InterventionCount=info.InterventionCount+double(intervention);

newGoalDistance=norm(info.Goal-info.Position);
progress=oldGoalDistance-newGoalDistance;
reward=8.0*progress-0.02*dot(nominalAcceleration,nominalAcceleration) ...
    -0.03*dot(safeAcceleration,safeAcceleration)-0.25*double(intervention) ...
    -2.0*slack;
if minimumSeparation<0.35
    reward=reward-3.0*(0.35-minimumSeparation);
end

info.Success=newGoalDistance<0.25 && norm(info.Velocity)<0.35;
info.Collision=minimumSeparation<0.0;
outOfBounds=abs(info.Position(1))>3.2 || abs(info.Position(2))>3.2 ...
    || info.Position(3)<0.15 || info.Position(3)>2.40;
if info.Success,reward=reward+30.0;end
if info.Collision,reward=reward-60.0;end
if outOfBounds,reward=reward-30.0;end
isDone=info.Success || info.Collision || outOfBounds ...
    || info.StepCount>=info.MaxSteps;
nextObservation=buildPPOObservationStage13(info);
end
