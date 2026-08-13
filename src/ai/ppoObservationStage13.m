function observation = ppoObservationStage13(xEstimate,trajectoryReference, ...
    previousSafeAcceleration,relativePosition,relativeVelocity,trackRadius, ...
    trackId,trackType,trackActive,gatTopEntityId,gatTopScore)
%PPOOBSERVATIONSTAGE13 Build the same 42-element observation in Simulink.

%#codegen

observation=zeros(42,1);
observation(1)=(trajectoryReference(1)-xEstimate(1))/3.0;
observation(2)=(trajectoryReference(2)-xEstimate(2))/3.0;
observation(3)=(trajectoryReference(3)-xEstimate(3))/3.0;
observation(4)=xEstimate(4)/2.0;
observation(5)=xEstimate(5)/2.0;
observation(6)=xEstimate(6)/2.0;
observation(7:9)=previousSafeAcceleration./[3;3;5];

writeIndex=10;
for rank=1:3
    selectedId=gatTopEntityId(rank);
    selectedTrack=0;
    for track=1:16
        if trackActive(track) && trackId(track)==selectedId
            selectedTrack=track;break;
        end
    end
    if selectedTrack>0
        observation(writeIndex:writeIndex+2)=relativePosition(:,selectedTrack)/3.0;
        observation(writeIndex+3:writeIndex+5)=relativeVelocity(:,selectedTrack)/2.0;
        observation(writeIndex+6)=trackRadius(selectedTrack);
        observation(writeIndex+7)=gatTopScore(rank);
        type=round(trackType(selectedTrack));
        if type>=1 && type<=3
            observation(writeIndex+7+type)=1.0;
        end
    end
    writeIndex=writeIndex+11;
end
observation=min(max(observation,-5.0),5.0);
end
