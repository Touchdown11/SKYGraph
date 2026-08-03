function [distanceM,valid,status,directionDistance,directionValid, ...
    nearestSensorId,hitEntityId,hitEntityType] = ...
    pairedToFArrayStage6(xTrue,t)
%PAIREDTOFARRAYSTAGE6 Clean paired front/left/rear/right ToF simulation.
%
% Physical mount pairs are 45 degrees apart around the body. Optical axes
% are only 20 degrees apart (+/-10 degrees around each cardinal direction)
% so a straight-ahead obstacle is not placed in a 20-degree blind gap.
%
% Ten-sensor order:
%  H0 front-right, H1 front-left,
%  H2 left-front, H3 left-rear,
%  H4 rear-left, H5 rear-right,
%  H6 right-rear, H7 right-front, H8 up, H9 down.
%
% Six direction outputs: front, left, rear, right, up, down.

%#codegen

minimumRange = 0.03;
maximumRange = 2.00;
halfFov = 12.5*pi/180.0;
numberOfSensors = 10;

% Physical origin angles: 45 degrees between the two mounts on each side.
mountAzimuthDeg = [-22.5,22.5,67.5,112.5,157.5,202.5,247.5,292.5];
% Beam axes: 20 degrees between each pair, centred on cardinal directions.
beamAzimuthDeg = [-10.0,10.0,80.0,100.0,170.0,190.0,260.0,280.0];
mountRadius = 0.10;

sensorDirectionBody = zeros(3,numberOfSensors);
sensorOriginBody = zeros(3,numberOfSensors);
for sensor = 1:8
    mountAngle = mountAzimuthDeg(sensor)*pi/180.0;
    beamAngle = beamAzimuthDeg(sensor)*pi/180.0;
    sensorOriginBody(:,sensor) = [ ...
        mountRadius*cos(mountAngle); ...
        mountRadius*sin(mountAngle); ...
        -0.04];
    sensorDirectionBody(:,sensor) = [cos(beamAngle);sin(beamAngle);0.0];
end
sensorOriginBody(:,9) = [0.0;0.0;0.06];
sensorOriginBody(:,10) = [0.0;0.0;-0.05];
sensorDirectionBody(:,9) = [0.0;0.0;1.0];
sensorDirectionBody(:,10) = [0.0;0.0;-1.0];

[entityPositionWorld,entityRadius,entityId,entityType,entityActive, ...
    groundZ,ceilingZ] = pairedSceneStage6(t);

positionWorld = xTrue(1:3);
RbodyToWorld = rotationStage6(xTrue(7),xTrue(8),xTrue(9));

distanceM = maximumRange*ones(numberOfSensors,1);
valid = false(numberOfSensors,1);
status = ones(numberOfSensors,1); % 0 valid, 1 no target, 2 too close
hitEntityId = zeros(numberOfSensors,1);
hitEntityType = zeros(numberOfSensors,1);

for sensor = 1:numberOfSensors
    directionWorld = RbodyToWorld*sensorDirectionBody(:,sensor);
    originWorld = positionWorld+RbodyToWorld*sensorOriginBody(:,sensor);
    bestRange = maximumRange;
    bestId = 0.0;
    bestType = 0.0;
    bestStatus = 1.0;
    found = false;

    for entity = 1:numel(entityRadius)
        if ~entityActive(entity)
            continue;
        end
        relative = entityPositionWorld(:,entity)-originWorld;
        centerRange = norm(relative);
        if centerRange < 1.0e-9
            continue;
        end
        forward = dot(relative,directionWorld);
        if forward <= 0.0
            continue;
        end
        cosineOffset = min(max(forward/centerRange,-1.0),1.0);
        angularOffset = acos(cosineOffset);
        angularRadius = asin(min(entityRadius(entity)/centerRange,1.0));
        if angularOffset <= halfFov+angularRadius
            candidate = centerRange-entityRadius(entity);
            candidateStatus = 0.0;
            if candidate < minimumRange
                candidate = minimumRange;
                candidateStatus = 2.0;
            end
            if candidate <= maximumRange && candidate < bestRange
                bestRange = candidate;
                bestId = entityId(entity);
                bestType = entityType(entity);
                bestStatus = candidateStatus;
                found = true;
            end
        end
    end

    % Central-ray intersections for floor and ceiling.
    verticalDirection = directionWorld(3);
    if abs(verticalDirection) > 1.0e-8
        planeRange = (groundZ-originWorld(3))/verticalDirection;
        if planeRange > 0.0 && planeRange <= maximumRange && planeRange < bestRange
            bestRange = max(planeRange,minimumRange);
            bestId = 9001.0;
            bestType = 4.0;
            bestStatus = double(planeRange < minimumRange)*2.0;
            found = true;
        end
        planeRange = (ceilingZ-originWorld(3))/verticalDirection;
        if planeRange > 0.0 && planeRange <= maximumRange && planeRange < bestRange
            bestRange = max(planeRange,minimumRange);
            bestId = 9002.0;
            bestType = 5.0;
            bestStatus = double(planeRange < minimumRange)*2.0;
            found = true;
        end
    end

    if found
        distanceM(sensor) = bestRange;
        valid(sensor) = true;
        status(sensor) = bestStatus;
        hitEntityId(sensor) = bestId;
        hitEntityType(sensor) = bestType;
    end
end

% Reduce the paired data to six dashboard/fallback directions.
directionDistance = maximumRange*ones(6,1);
directionValid = false(6,1);
nearestSensorId = -ones(6,1);
pairs = [1,2;3,4;5,6;7,8;9,9;10,10];
for direction = 1:6
    for sensor = pairs(direction,1):pairs(direction,2)
        if valid(sensor) && distanceM(sensor) < directionDistance(direction)
            directionDistance(direction) = distanceM(sensor);
            directionValid(direction) = true;
            nearestSensorId(direction) = sensor-1;
        end
    end
end
end

function R = rotationStage6(roll,pitch,yaw)
cR = cos(roll);
sR = sin(roll);
cP = cos(pitch);
sP = sin(pitch);
cY = cos(yaw);
sY = sin(yaw);
R = [ ...
    cY*cP,cY*sP*sR-sY*cR,cY*sP*cR+sY*sR; ...
    sY*cP,sY*sP*sR+cY*cR,sY*sP*cR-cY*sR; ...
    -sP,cP*sR,cP*cR];
end
