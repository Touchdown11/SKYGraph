function world=buildMultiEgoWorldStage16(egoState,configuration,time)
%BUILDMULTIEGOWORLDSTAGE16 Publish controlled egos and fixed world objects.
% IDs 101.. are controlled cooperative egos; 201.. are mapped structures.
maximumEntities=16;
numberOfEgos=size(egoState,2);
world.position=zeros(3,maximumEntities); world.velocity=zeros(3,maximumEntities);
world.radius=zeros(maximumEntities,1); world.id=zeros(maximumEntities,1);
world.type=zeros(maximumEntities,1); world.active=false(maximumEntities,1);
world.telemetry=false(maximumEntities,1); world.mapAvailable=false(maximumEntities,1);
world.groundZ=0.0; world.ceilingZ=2.5;
for ego=1:numberOfEgos
    world.position(:,ego)=egoState(1:3,ego); world.velocity(:,ego)=egoState(4:6,ego);
    world.radius(ego)=configuration.EgoRadius; world.id(ego)=100+ego;
    world.type(ego)=1; world.active(ego)=true; world.telemetry(ego)=configuration.TelemetryEnabled;
end
% Structures are deliberately held in the same global table as peers.
if configuration.Scenario=="dense"
    locations=[1.7,-1.6,-1.5,1.6;1.6,1.7,-1.6,-1.6;1,1,1,1];
    radii=[.38,.32,.36,.34];
    for k=1:4
        slot=numberOfEgos+k;
        if slot>maximumEntities,break,end
        world.position(:,slot)=locations(:,k); world.radius(slot)=radii(k);
        world.id(slot)=200+k; world.type(slot)=2; world.active(slot)=true; world.mapAvailable(slot)=true;
    end
end
% A deterministic non-cooperative intruder can be included in dense runs.
if configuration.Scenario=="dense" && numberOfEgos+5<=maximumEntities
    slot=numberOfEgos+5;
    world.position(:,slot)=[1.2*sin(.18*time);.35;1.0];
    world.velocity(:,slot)=[1.2*.18*cos(.18*time);0;0];
    world.radius(slot)=.12; world.id(slot)=301; world.type(slot)=3; world.active(slot)=true;
end
end
