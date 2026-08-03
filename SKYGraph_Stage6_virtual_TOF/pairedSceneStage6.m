function [entityPositionWorld,entityRadius,entityId,entityType, ...
    entityActive,groundZ,ceilingZ] = pairedSceneStage6(t)
%PAIREDSCENESTAGE6 Fixed scene for the clean paired-sensor model.
% Types: 1 cooperative drone, 2 building, 3 unknown obstacle.

%#codegen

time = t(1);
maximumEntities = 6;
entityPositionWorld = zeros(3,maximumEntities);
entityRadius = zeros(maximumEntities,1);
entityId = zeros(maximumEntities,1);
entityType = zeros(maximumEntities,1);
entityActive = false(maximumEntities,1);
groundZ = 0.0;
ceilingZ = 2.5;

% One object centred in each cardinal detection direction.
entityPositionWorld(:,1) = [1.35;0.00;1.00];
entityRadius(1) = 0.12;
entityId(1) = 201;
entityType(1) = 2;
entityActive(1) = true;

entityPositionWorld(:,2) = [0.00;1.30+0.08*sin(0.25*time);1.05];
entityRadius(2) = 0.14;
entityId(2) = 101;
entityType(2) = 1;
entityActive(2) = true;

entityPositionWorld(:,3) = [-1.25;0.00;0.95];
entityRadius(3) = 0.11;
entityId(3) = 301;
entityType(3) = 3;
entityActive(3) = true;

entityPositionWorld(:,4) = [0.00;-1.40;1.00];
entityRadius(4) = 0.15;
entityId(4) = 202;
entityType(4) = 2;
entityActive(4) = true;

% Object above the vehicle.
entityPositionWorld(:,5) = [0.05;0.00;1.85];
entityRadius(5) = 0.10;
entityId(5) = 302;
entityType(5) = 3;
entityActive(5) = true;
end
