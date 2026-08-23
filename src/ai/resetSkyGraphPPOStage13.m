function [initialObservation,info] = resetSkyGraphPPOStage13()
%RESETSKYGRAPHPPOSTAGE13 Randomized safe-navigation episode.

info=struct;
info.Position=[0;0;1.0];
info.Velocity=zeros(3,1);
angle=2*pi*rand;
goalRadius=1.8+0.7*rand;
info.Goal=[goalRadius*cos(angle);goalRadius*sin(angle);0.7+0.7*rand];
info.ObstaclePosition=zeros(3,8);
info.ObstacleVelocity=zeros(3,8);
info.ObstacleRadius=zeros(8,1);
info.ObstacleType=zeros(8,1);
info.ObstacleActive=false(8,1);
obstacleCount=randi([3,8]);
for k=1:obstacleCount
    accepted=false;
    while ~accepted
        candidate=[-2.4+4.8*rand;-2.4+4.8*rand;0.35+1.35*rand];
        accepted=norm(candidate-info.Position)>0.65 && norm(candidate-info.Goal)>0.55;
    end
    info.ObstaclePosition(:,k)=candidate;
    info.ObstacleRadius(k)=0.10+0.22*rand;
    info.ObstacleType(k)=randi(3);
    info.ObstacleActive(k)=true;
    if info.ObstacleType(k)==2
        info.ObstacleVelocity(:,k)=zeros(3,1);
    else
        direction=randn(3,1);direction(3)=0.25*direction(3);
        direction=direction/max(norm(direction),1e-6);
        info.ObstacleVelocity(:,k)=(0.10+0.35*rand)*direction;
    end
end
info.PreviousSafeAcceleration=zeros(3,1);
info.StepCount=0;
info.MaxSteps=200;
info.InterventionCount=0;
info.MinimumSeparation=100;
info.Collision=false;
info.Success=false;
initialObservation=buildPPOObservationStage13(info);
end
