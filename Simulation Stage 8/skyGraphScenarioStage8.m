function [entityPositionWorld,entityVelocityWorld,entityRadius, ...
    entityId,entityType,entityActive,telemetryAvailable,mapAvailable, ...
    groundZ,ceilingZ,ambientMode,faultMode,scenarioPhase] = ...
    skyGraphScenarioStage8(t,scenarioMode,faultScheduleEnabled)
%SKYGRAPHSCENARIOSTAGE8 Fixed-capacity multi-entity scenario generator.
%
% Types: 1 cooperative drone, 2 building, 3 unknown/rogue obstacle.
% Modes: 1 cardinal mixed, 2 crossing traffic, 3 diagonal blind entry,
%        4 vertical threats, 5 dense mixed urban scene.

%#codegen

maximumEntities = 16;
time = t(1);
mode = round(scenarioMode(1));
useFaultSchedule = faultScheduleEnabled(1) > 0.5;

entityPositionWorld = zeros(3,maximumEntities);
entityVelocityWorld = zeros(3,maximumEntities);
entityRadius = zeros(maximumEntities,1);
entityId = zeros(maximumEntities,1);
entityType = zeros(maximumEntities,1);
entityActive = false(maximumEntities,1);
telemetryAvailable = false(maximumEntities,1);
mapAvailable = false(maximumEntities,1);
groundZ = 0.0;
ceilingZ = 2.5;

if mode == 2
    % Cooperative drone crossing from right to left in front of ego.
    entityPositionWorld(:,1) = [1.00;1.35*sin(0.22*time);1.10];
    entityVelocityWorld(:,1) = [0.0;1.35*0.22*cos(0.22*time);0.0];
    entityRadius(1) = 0.16; entityId(1) = 101; entityType(1) = 1;
    entityActive(1) = true; telemetryAvailable(1) = true;

    % Non-cooperative head-on intruder oscillating through front/rear.
    entityPositionWorld(:,2) = [1.55*cos(0.14*time);0.18;1.00];
    entityVelocityWorld(:,2) = [-1.55*0.14*sin(0.14*time);0.0;0.0];
    entityRadius(2) = 0.14; entityId(2) = 301; entityType(2) = 3;
    entityActive(2) = true;

    entityPositionWorld(:,3) = [0.0;1.75;1.0];
    entityRadius(3) = 0.35; entityId(3) = 201; entityType(3) = 2;
    entityActive(3) = true; mapAvailable(3) = true;

elseif mode == 3
    % Narrow unknown obstacle stays on the exact 45-degree diagonal and
    % slowly approaches the origin through the paired layout's blind area.
    range = max(0.75,2.10-0.035*time);
    entityPositionWorld(:,1) = range*[sqrt(0.5);sqrt(0.5);0.0]+[0;0;1.0];
    entityVelocityWorld(:,1) = -0.035*[sqrt(0.5);sqrt(0.5);0.0];
    entityRadius(1) = 0.055; entityId(1) = 399; entityType(1) = 3;
    entityActive(1) = true;

    % Known building visible in the front pair for reference.
    entityPositionWorld(:,2) = [1.45;0.0;1.0];
    entityRadius(2) = 0.22; entityId(2) = 201; entityType(2) = 2;
    entityActive(2) = true; mapAvailable(2) = true;

elseif mode == 4
    % Vertical cooperative drone descending from above.
    entityPositionWorld(:,1) = [0.05;0.0;2.05-0.25*sin(0.15*time)];
    entityVelocityWorld(:,1) = [0.0;0.0;-0.25*0.15*cos(0.15*time)];
    entityRadius(1) = 0.15; entityId(1) = 102; entityType(1) = 1;
    entityActive(1) = true; telemetryAvailable(1) = true;

    % Unknown object moving below the ego test altitude.
    entityPositionWorld(:,2) = [-0.08;0.0;0.30+0.12*sin(0.20*time)];
    entityVelocityWorld(:,2) = [0.0;0.0;0.12*0.20*cos(0.20*time)];
    entityRadius(2) = 0.09; entityId(2) = 303; entityType(2) = 3;
    entityActive(2) = true;

elseif mode == 5
    % Four mapped structures.
    buildingPosition = [1.75,-1.65,-1.55,1.60;1.60,1.70,-1.65,-1.60;1.0,1.0,1.0,1.0];
    buildingRadius = [0.38,0.32,0.36,0.34];
    for k = 1:4
        entityPositionWorld(:,k) = buildingPosition(:,k);
        entityRadius(k) = buildingRadius(k);
        entityId(k) = 200+k;
        entityType(k) = 2;
        entityActive(k) = true;
        mapAvailable(k) = true;
    end

    % Three cooperative drones.
    for k = 1:3
        phase = (k-1)*2*pi/3;
        rate = 0.12+0.02*k;
        radiusPath = 1.00+0.12*k;
        index = 4+k;
        entityPositionWorld(:,index) = [ ...
            radiusPath*cos(rate*time+phase); ...
            radiusPath*sin(rate*time+phase); ...
            0.90+0.18*k];
        entityVelocityWorld(:,index) = [ ...
            -radiusPath*rate*sin(rate*time+phase); ...
             radiusPath*rate*cos(rate*time+phase); ...
             0.0];
        entityRadius(index) = 0.14;
        entityId(index) = 100+k;
        entityType(index) = 1;
        entityActive(index) = true;
        telemetryAvailable(index) = true;
    end

    % Two unknown moving entities.
    entityPositionWorld(:,8) = [1.20*sin(0.18*time);0.35;1.00];
    entityVelocityWorld(:,8) = [1.20*0.18*cos(0.18*time);0.0;0.0];
    entityRadius(8) = 0.12; entityId(8) = 301; entityType(8) = 3;
    entityActive(8) = true;

    entityPositionWorld(:,9) = [-0.40;1.15*cos(0.16*time);0.80];
    entityVelocityWorld(:,9) = [0.0;-1.15*0.16*sin(0.16*time);0.0];
    entityRadius(9) = 0.10; entityId(9) = 302; entityType(9) = 3;
    entityActive(9) = true;

else
    % Mode 1: one entity centred in each cardinal direction plus overhead.
    positions = [1.35,0.0,-1.25,0.0,0.05;0.0,1.30,0.0,-1.40,0.0;1.0,1.05,0.95,1.0,1.85];
    radii = [0.12,0.14,0.11,0.15,0.10];
    ids = [201,101,301,202,302];
    types = [2,1,3,2,3];
    for k = 1:5
        entityPositionWorld(:,k) = positions(:,k);
        entityRadius(k) = radii(k);
        entityId(k) = ids(k);
        entityType(k) = types(k);
        entityActive(k) = true;
    end
    telemetryAvailable(2) = true;
    mapAvailable(1) = true;
    mapAvailable(4) = true;
end

% Repeatable scheduled environment/sensor/channel stress sequence.
ambientMode = 0.0;
faultMode = 0.0;
scenarioPhase = 0.0;
if useFaultSchedule
    if time < 4.0
        scenarioPhase = 0.0;
    elseif time < 8.0
        ambientMode = 1.0; scenarioPhase = 1.0;
    elseif time < 12.0
        faultMode = 1.0; scenarioPhase = 2.0;
    elseif time < 16.0
        faultMode = 2.0; scenarioPhase = 3.0;
    elseif time < 20.0
        faultMode = 3.0; scenarioPhase = 4.0;
    elseif time < 24.0
        faultMode = 4.0; scenarioPhase = 5.0;
    elseif time < 27.0
        faultMode = 5.0; scenarioPhase = 6.0;
    else
        scenarioPhase = 7.0;
    end
end
end
