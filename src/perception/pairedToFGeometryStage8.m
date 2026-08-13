function [distanceM,valid,status,hitEntityId,hitEntityType] = ...
    pairedToFGeometryStage8(xTrue,entityPositionWorld,entityRadius, ...
    entityId,entityType,entityActive,groundZ,ceilingZ)
%PAIREDTOFGEOMETRYSTAGE8 Ideal paired sensor geometry for external scenes.

%#codegen

minimumRange = 0.03;
maximumRange = 2.0;
halfFov = 12.5*pi/180.0;
mountAzimuthDeg = [-22.5,22.5,67.5,112.5,157.5,202.5,247.5,292.5];
beamAzimuthDeg = [-10,10,80,100,170,190,260,280];
mountRadius = 0.10;

sensorOriginBody = zeros(3,10);
sensorDirectionBody = zeros(3,10);
for sensor = 1:8
    mountAngle = mountAzimuthDeg(sensor)*pi/180.0;
    beamAngle = beamAzimuthDeg(sensor)*pi/180.0;
    sensorOriginBody(:,sensor) = [mountRadius*cos(mountAngle); ...
        mountRadius*sin(mountAngle);-0.04];
    sensorDirectionBody(:,sensor) = [cos(beamAngle);sin(beamAngle);0.0];
end
sensorOriginBody(:,9) = [0;0;0.06];
sensorOriginBody(:,10) = [0;0;-0.05];
sensorDirectionBody(:,9) = [0;0;1];
sensorDirectionBody(:,10) = [0;0;-1];

R = rotationStage8(xTrue(7),xTrue(8),xTrue(9));
positionWorld = xTrue(1:3);
distanceM = maximumRange*ones(10,1);
valid = false(10,1);
status = ones(10,1);
hitEntityId = zeros(10,1);
hitEntityType = zeros(10,1);

for sensor = 1:10
    directionWorld = R*sensorDirectionBody(:,sensor);
    originWorld = positionWorld+R*sensorOriginBody(:,sensor);
    bestRange = maximumRange;
    bestId = 0.0;
    bestType = 0.0;
    bestStatus = 1.0;
    found = false;

    for entity = 1:16
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
        angularOffset = acos(min(max(forward/centerRange,-1.0),1.0));
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

    verticalDirection = directionWorld(3);
    if abs(verticalDirection) > 1.0e-8
        planeRange = (groundZ-originWorld(3))/verticalDirection;
        if planeRange > 0 && planeRange <= maximumRange && planeRange < bestRange
            bestRange = max(planeRange,minimumRange);
            bestId = 9001; bestType = 4;
            bestStatus = double(planeRange < minimumRange)*2.0;
            found = true;
        end
        planeRange = (ceilingZ-originWorld(3))/verticalDirection;
        if planeRange > 0 && planeRange <= maximumRange && planeRange < bestRange
            bestRange = max(planeRange,minimumRange);
            bestId = 9002; bestType = 5;
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
end

function R = rotationStage8(roll,pitch,yaw)
cR=cos(roll);sR=sin(roll);cP=cos(pitch);sP=sin(pitch);cY=cos(yaw);sY=sin(yaw);
R=[cY*cP,cY*sP*sR-sY*cR,cY*sP*cR+sY*sR; ...
   sY*cP,sY*sP*sR+cY*cR,sY*sP*cR-cY*sR; ...
   -sP,cP*sR,cP*cR];
end
