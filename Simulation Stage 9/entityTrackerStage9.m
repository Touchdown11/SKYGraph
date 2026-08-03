function [trackPositionWorld,trackVelocityWorld,trackRelativePosition, ...
    trackRelativeVelocity,trackRadius,trackId,trackType,trackActive, ...
    trackConfidence,trackAge,trackSourceMask,closingSpeed,timeToCollision] = ...
    entityTrackerStage9(xEstimate,distanceM,valid,status,packetNew,packetAge, ...
    sourcePositionWorld,sourceVelocityWorld,sourceRadius,sourceId, ...
    sourceActive,telemetryAvailable,mapAvailable)
%ENTITYTRACKERSTAGE9 Fixed-capacity sector tracking and source association.
%
% Deployed type rules:
%   type 1 cooperative: authenticated telemetry source
%   type 2 building: mapped source
%   type 3 unknown: unassociated ToF track
%
% Source mask bits: 1 telemetry, 2 map, 4 ToF.
% Ground-truth scenario type is intentionally not an input.

%#codegen

persistent initialized sensorTrackPosition sensorTrackVelocity ...
    sensorTrackActive sensorTrackAge sensorTrackConfidence

Ts = 0.05;
maximumTracks = 16;
numberOfSensors = 10;
alpha = 0.65;
beta = 0.18;
maximumTrackAge = 0.50;

if isempty(initialized)
    sensorTrackPosition = zeros(3,numberOfSensors);
    sensorTrackVelocity = zeros(3,numberOfSensors);
    sensorTrackActive = false(numberOfSensors,1);
    sensorTrackAge = zeros(numberOfSensors,1);
    sensorTrackConfidence = zeros(numberOfSensors,1);
    initialized = true;
end

% Explicit scalar assignments preserve column-vector orientation even if an
% upstream Simulink signal is displayed as a one-dimensional vector.
egoPosition = zeros(3,1);
egoVelocity = zeros(3,1);
egoPosition(1) = xEstimate(1);
egoPosition(2) = xEstimate(2);
egoPosition(3) = xEstimate(3);
egoVelocity(1) = xEstimate(4);
egoVelocity(2) = xEstimate(5);
egoVelocity(3) = xEstimate(6);
RbodyToWorld = rotationStage9(xEstimate(7),xEstimate(8),xEstimate(9));

% Paired sensor physical origins and beam axes.
mountAzimuthDeg = [-22.5,22.5,67.5,112.5,157.5,202.5,247.5,292.5];
beamAzimuthDeg = [-10,10,80,100,170,190,260,280];
sensorOriginBody = zeros(3,numberOfSensors);
sensorDirectionBody = zeros(3,numberOfSensors);
for sensor = 1:8
    sensorOriginBody(:,sensor) = [0.10*cos(mountAzimuthDeg(sensor)*pi/180.0); ...
        0.10*sin(mountAzimuthDeg(sensor)*pi/180.0);-0.04];
    sensorDirectionBody(:,sensor) = [cos(beamAzimuthDeg(sensor)*pi/180.0); ...
        sin(beamAzimuthDeg(sensor)*pi/180.0);0.0];
end
sensorOriginBody(:,9) = [0;0;0.06];
sensorOriginBody(:,10) = [0;0;-0.05];
sensorDirectionBody(:,9) = [0;0;1];
sensorDirectionBody(:,10) = [0;0;-1];

frameFresh = packetAge <= 0.15;
for sensor = 1:numberOfSensors
    % Predict every 20 Hz tracker step.
    if sensorTrackActive(sensor)
        sensorTrackPosition(:,sensor) = sensorTrackPosition(:,sensor) ...
            +Ts*sensorTrackVelocity(:,sensor);
        sensorTrackAge(sensor) = sensorTrackAge(sensor)+Ts;
        sensorTrackConfidence(sensor) = max(0.0, ...
            sensorTrackConfidence(sensor)-0.04);
    end

    usableMeasurement = packetNew && frameFresh && valid(sensor) ...
        && (status(sensor) == 0.0 || status(sensor) == 2.0);
    if usableMeasurement
        pointBody = sensorOriginBody(:,sensor) ...
            +distanceM(sensor)*sensorDirectionBody(:,sensor);
        measurementWorld = egoPosition+RbodyToWorld*pointBody;
        if sensorTrackActive(sensor)
            residual = measurementWorld-sensorTrackPosition(:,sensor);
            sensorTrackPosition(:,sensor) = sensorTrackPosition(:,sensor) ...
                +alpha*residual;
            sensorTrackVelocity(:,sensor) = sensorTrackVelocity(:,sensor) ...
                +(beta/Ts)*residual;
        else
            sensorTrackPosition(:,sensor) = measurementWorld;
            sensorTrackVelocity(:,sensor) = zeros(3,1);
            sensorTrackActive(sensor) = true;
            sensorTrackConfidence(sensor) = 0.35;
        end
        sensorTrackAge(sensor) = 0.0;
        sensorTrackConfidence(sensor) = min(1.0, ...
            sensorTrackConfidence(sensor)+0.18);
    end

    if sensorTrackActive(sensor) && (sensorTrackAge(sensor) > maximumTrackAge ...
            || sensorTrackConfidence(sensor) < 0.05)
        sensorTrackActive(sensor) = false;
        sensorTrackConfidence(sensor) = 0.0;
    end
end

% Fixed-capacity output table.
trackPositionWorld = zeros(3,maximumTracks);
trackVelocityWorld = zeros(3,maximumTracks);
trackRadius = zeros(maximumTracks,1);
trackId = zeros(maximumTracks,1);
trackType = zeros(maximumTracks,1);
trackActive = false(maximumTracks,1);
trackConfidence = zeros(maximumTracks,1);
trackAge = zeros(maximumTracks,1);
trackSourceMask = zeros(maximumTracks,1);
trackCount = 0;

% First insert active ToF tracks, associated only with permitted sources.
for sensor = 1:numberOfSensors
    if ~sensorTrackActive(sensor)
        continue;
    end
    candidatePosition = sensorTrackPosition(:,sensor);
    candidateVelocity = sensorTrackVelocity(:,sensor);
    candidateRadius = 0.15;
    candidateId = 1000.0+(sensor-1);
    candidateType = 3.0;
    candidateMask = 4.0;
    candidateConfidence = sensorTrackConfidence(sensor);
    candidateAge = sensorTrackAge(sensor);

    [matched,index,matchedType,matchedMask] = associateSourceStage9( ...
        candidatePosition,sourcePositionWorld,sourceRadius,sourceActive, ...
        telemetryAvailable,mapAvailable);
    if matched
        candidatePosition = sourcePositionWorld(:,index);
        candidateVelocity = sourceVelocityWorld(:,index);
        candidateRadius = sourceRadius(index);
        candidateId = sourceId(index);
        candidateType = matchedType;
        candidateMask = 4.0+matchedMask;
        candidateConfidence = max(candidateConfidence,0.90);
    end

    existing = findTrackByIdStage9(trackId,trackActive,candidateId);
    if existing > 0
        % Merge multiple sensor sectors that see the same source entity.
        trackPositionWorld(:,existing) = 0.5*( ...
            trackPositionWorld(:,existing)+candidatePosition);
        trackVelocityWorld(:,existing) = 0.5*( ...
            trackVelocityWorld(:,existing)+candidateVelocity);
        trackConfidence(existing) = max(trackConfidence(existing),candidateConfidence);
        trackAge(existing) = min(trackAge(existing),candidateAge);
        trackSourceMask(existing) = max(trackSourceMask(existing),candidateMask);
    elseif trackCount < maximumTracks
        trackCount = trackCount+1;
        trackPositionWorld(:,trackCount) = candidatePosition;
        trackVelocityWorld(:,trackCount) = candidateVelocity;
        trackRadius(trackCount) = candidateRadius;
        trackId(trackCount) = candidateId;
        trackType(trackCount) = candidateType;
        trackActive(trackCount) = true;
        trackConfidence(trackCount) = candidateConfidence;
        trackAge(trackCount) = candidateAge;
        trackSourceMask(trackCount) = candidateMask;
    end
end

% Add telemetry/map entities even if not currently inside a ToF cone.
for source = 1:16
    if ~sourceActive(source) || ~(telemetryAvailable(source) || mapAvailable(source))
        continue;
    end
    if norm(sourcePositionWorld(:,source)-egoPosition) > 3.0
        continue;
    end
    existing = findTrackByIdStage9(trackId,trackActive,sourceId(source));
    if existing > 0
        continue;
    end
    if trackCount >= maximumTracks
        break;
    end
    trackCount = trackCount+1;
    trackPositionWorld(:,trackCount) = sourcePositionWorld(:,source);
    trackVelocityWorld(:,trackCount) = sourceVelocityWorld(:,source);
    trackRadius(trackCount) = sourceRadius(source);
    trackId(trackCount) = sourceId(source);
    trackActive(trackCount) = true;
    trackAge(trackCount) = 0.0;
    if telemetryAvailable(source)
        trackType(trackCount) = 1.0;
        trackSourceMask(trackCount) = 1.0;
        trackConfidence(trackCount) = 0.90;
    else
        trackType(trackCount) = 2.0;
        trackSourceMask(trackCount) = 2.0;
        trackConfidence(trackCount) = 0.95;
    end
end

% Relative state and threat kinematics.
trackRelativePosition = zeros(3,maximumTracks);
trackRelativeVelocity = zeros(3,maximumTracks);
closingSpeed = zeros(maximumTracks,1);
timeToCollision = 1000.0*ones(maximumTracks,1);
for track = 1:maximumTracks
    if ~trackActive(track)
        continue;
    end
    relativePosition = zeros(3,1);
    relativeVelocity = zeros(3,1);
    relativePosition(1) = trackPositionWorld(1,track)-egoPosition(1);
    relativePosition(2) = trackPositionWorld(2,track)-egoPosition(2);
    relativePosition(3) = trackPositionWorld(3,track)-egoPosition(3);
    relativeVelocity(1) = trackVelocityWorld(1,track)-egoVelocity(1);
    relativeVelocity(2) = trackVelocityWorld(2,track)-egoVelocity(2);
    relativeVelocity(3) = trackVelocityWorld(3,track)-egoVelocity(3);
    trackRelativePosition(:,track) = relativePosition;
    trackRelativeVelocity(:,track) = relativeVelocity;
    centreDistance = sqrt(relativePosition(1)^2+relativePosition(2)^2 ...
        +relativePosition(3)^2);
    centreDistance = max(centreDistance,1.0e-6);
    separation = centreDistance-trackRadius(track);
    closing = -(relativePosition(1)*relativeVelocity(1) ...
        +relativePosition(2)*relativeVelocity(2) ...
        +relativePosition(3)*relativeVelocity(3))/centreDistance;
    closingSpeed(track) = closing;
    if closing > 0.01
        safeSeparation = max(separation,0.0);
        timeToCollision(track) = safeSeparation/closing;
    end
end
end

function [matched,index,type,mask] = associateSourceStage9( ...
    point,sourcePosition,sourceRadius,sourceActive,telemetry,mapAvailable)
matched = false;
index = 0;
type = 3.0;
mask = 0.0;
bestError = 0.30;
for source = 1:16
    if ~sourceActive(source) || ~(telemetry(source) || mapAvailable(source))
        continue;
    end
    surfaceError = abs(norm(point-sourcePosition(:,source))-sourceRadius(source));
    if surfaceError < bestError
        bestError = surfaceError;
        matched = true;
        index = source;
        if telemetry(source)
            type = 1.0;
            mask = 1.0;
        else
            type = 2.0;
            mask = 2.0;
        end
    end
end
end

function index = findTrackByIdStage9(ids,active,id)
index = 0;
for k = 1:numel(ids)
    if active(k) && ids(k) == id
        index = k;
        return;
    end
end
end

function R = rotationStage9(roll,pitch,yaw)
cR=cos(roll);sR=sin(roll);cP=cos(pitch);sP=sin(pitch);cY=cos(yaw);sY=sin(yaw);
R=[cY*cP,cY*sP*sR-sY*cR,cY*sP*cR+sY*sR; ...
   sY*cP,sY*sP*sR+cY*cR,sY*sP*cR-cY*sR; ...
   -sP,cP*sR,cP*cR];
end
