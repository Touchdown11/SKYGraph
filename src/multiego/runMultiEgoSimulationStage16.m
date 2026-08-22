function output=runMultiEgoSimulationStage16(configuration)
%RUNMULTIEGOSIMULATIONSTAGE16 Decentralized fixed-step multi-ego simulation.
% Each ego independently builds a local SkyGraph and projects its waypoint
% acceleration through the existing Stage-11 CBF. Dynamics are advanced
% simultaneously, so no ego receives another ego's future state.
if nargin<1,configuration=configureMultiEgoStage16();else,configuration=configureMultiEgoStage16(configuration);end
rng(configuration.RandomSeed,'twister');
N=configuration.NumberOfEgos; dt=configuration.SampleTime; time=(0:dt:configuration.Duration); K=numel(time);
[state,goal]=initialConditions16(N,configuration); initialState=state;
position=zeros(3,N,K); velocity=position; safeAcceleration=position; nominalAcceleration=position;
minimumSeparation=inf(1,K); collision=false(1,K); cbfIntervention=false(N,K); graphEdges=zeros(N,K); graphNodes=zeros(N,K); topThreat=zeros(N,K);
for step=1:K
    world=buildMultiEgoWorldStage16(state,configuration,time(step));
    command=zeros(3,N);
    for ego=1:N
        tracks=multiEgoTrackerStage16(ego,state(:,ego),world,configuration);
        [~,mask,~,~,adjacency,edgeCount,~,~,~,topScore]=graphBuilderStage10( ...
            tracks.relativePosition,tracks.relativeVelocity,tracks.radius,tracks.id,tracks.type, ...
            tracks.active,tracks.confidence,tracks.age,tracks.sourceMask,tracks.closingSpeed,tracks.timeToCollision);
        % A goal-seeking baseline is deliberately used unless a separately
        % trained multi-ego PPO policy has been attached by an integration model.
        nominal=2.0*(goal(:,ego)-state(1:3,ego))-1.25*state(4:6,ego);
        nominal=min(max(nominal,[-3;-3;-4]),[3;3;4]);
        [safe,intervention]=cbfSafetyShieldStage11(nominal,state(4:6,ego), ...
            tracks.relativePosition,tracks.relativeVelocity,tracks.radius,tracks.active,tracks.confidence,tracks.age);
        command(:,ego)=safe; nominalAcceleration(:,ego,step)=nominal; safeAcceleration(:,ego,step)=safe;
        cbfIntervention(ego,step)=intervention; graphEdges(ego,step)=edgeCount; graphNodes(ego,step)=sum(mask); topThreat(ego,step)=max(topScore);
    end
    position(:,:,step)=state(1:3,:); velocity(:,:,step)=state(4:6,:);
    [minimumSeparation(step),collision(step)]=pairwiseSafety16(state(1:3,:),configuration.EgoRadius);
    if step<K
        % Simultaneous semi-implicit point-mass integration with bounded speed.
        nextVelocity=state(4:6,:)+dt*command;
        speed=sqrt(sum(nextVelocity.^2,1)); scale=max(1,speed/2.5); nextVelocity=nextVelocity./scale;
        state(1:3,:)=state(1:3,:)+dt*nextVelocity; state(4:6,:)=nextVelocity;
        state(3,:)=min(max(state(3,:),configuration.EgoRadius),2.5-configuration.EgoRadius);
    end
end
finalError=squeeze(sqrt(sum((position(:,:,end)-goal).^2,1)));
output=struct('Configuration',configuration,'Time',time,'InitialState',initialState,'Goal',goal, ...
    'Position',position,'Velocity',velocity,'NominalAcceleration',nominalAcceleration, ...
    'SafeAcceleration',safeAcceleration,'MinimumPairwiseSeparation',minimumSeparation, ...
    'Collision',collision,'CBFIntervention',cbfIntervention,'GraphEdges',graphEdges, ...
    'GraphNodes',graphNodes,'TopThreatScore',topThreat,'FinalGoalError',finalError, ...
    'MissionSuccess',all(finalError<.25) && ~any(collision));
if strlength(string(configuration.OutputFile))>0,save(configuration.OutputFile,'output');end
if configuration.Visualize,plotMultiEgoStage16(output);end
end

function [state,goal]=initialConditions16(N,c)
state=zeros(6,N); goal=zeros(3,N);
for ego=1:N
    switch c.Scenario
        case "headon"
            signDirection=(-1)^(ego+1); state(:,ego)=[1.4*signDirection;0;1;0;0;0]; goal(:,ego)=[-1.4*signDirection;0;1];
        case "merge"
            state(:,ego)=[-1.5;(-1)^(ego)*(.25+.18*ego);.8+.08*mod(ego,2);0;0;0]; goal(:,ego)=[1.5;.30*(-1)^(ego);1];
        otherwise % crossing and dense: phase-spaced traversal of the centre
            angle=2*pi*(ego-1)/N; state(:,ego)=[1.5*cos(angle);1.5*sin(angle);.85+.12*mod(ego,3);0;0;0]; goal(:,ego)=-state(1:3,ego);
    end
end
end

function [minimum,hit]=pairwiseSafety16(position,radius)
minimum=inf; hit=false; N=size(position,2);
for first=1:N-1
    for second=first+1:N
        separation=norm(position(:,first)-position(:,second))-2*radius;
        minimum=min(minimum,separation); hit=hit || separation<0;
    end
end
end
