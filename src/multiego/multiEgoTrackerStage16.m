function tracks=multiEgoTrackerStage16(egoIndex,egoState,world,configuration)
%MULTIEGOTRACKERSTAGE16 Independent ego-centric table from local permitted sources.
% This truth-backed cooperative/map adapter intentionally models only sources
% available through telemetry or maps. ToF-only tracks remain the responsibility
% of the Stage-9 sensor tracker when integrating this runner into Simulink.
maximumTracks=16;
tracks.position=zeros(3,maximumTracks); tracks.velocity=zeros(3,maximumTracks);
tracks.relativePosition=zeros(3,maximumTracks); tracks.relativeVelocity=zeros(3,maximumTracks);
tracks.radius=zeros(maximumTracks,1); tracks.id=zeros(maximumTracks,1); tracks.type=zeros(maximumTracks,1);
tracks.active=false(maximumTracks,1); tracks.confidence=zeros(maximumTracks,1); tracks.age=zeros(maximumTracks,1);
tracks.sourceMask=zeros(maximumTracks,1); tracks.closingSpeed=zeros(maximumTracks,1); tracks.timeToCollision=1000*ones(maximumTracks,1);
count=0; egoPosition=egoState(1:3); egoVelocity=egoState(4:6);
for source=1:numel(world.id)
    if source==egoIndex || ~world.active(source),continue,end
    permitted=world.mapAvailable(source) || (world.telemetry(source) && rand>=configuration.TelemetryDropoutProbability);
    if ~permitted || norm(world.position(:,source)-egoPosition)>3.5,continue,end
    if count>=maximumTracks,break,end
    count=count+1; tracks.position(:,count)=world.position(:,source); tracks.velocity(:,count)=world.velocity(:,source);
    tracks.radius(count)=world.radius(source); tracks.id(count)=world.id(source); tracks.type(count)=world.type(source);
    tracks.active(count)=true; tracks.confidence(count)=world.mapAvailable(source)*.95+world.telemetry(source)*.90;
    tracks.sourceMask(count)=world.mapAvailable(source)*2+world.telemetry(source);
    r=tracks.position(:,count)-egoPosition; v=tracks.velocity(:,count)-egoVelocity;
    tracks.relativePosition(:,count)=r; tracks.relativeVelocity(:,count)=v;
    d=max(norm(r),1e-6); tracks.closingSpeed(count)=-dot(r,v)/d;
    if tracks.closingSpeed(count)>.01,tracks.timeToCollision(count)=max(d-tracks.radius(count)-configuration.EgoRadius,0)/tracks.closingSpeed(count);end
end
end
