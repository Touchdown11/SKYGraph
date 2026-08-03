%% TESTSCENARIOSTAGE8 Validate fixed-capacity scenes and fault schedule
clear;
clc;
close all;

expectedActive = [5,3,2,2,9];
for mode = 1:5
    [position,velocity,radius,id,type,active,telemetry,map,ground,ceiling] = ...
        skyGraphScenarioStage8(0.0,mode,0.0); %#ok<ASGLU>
    assert(isequal(size(position),[3,16]));
    assert(isequal(size(velocity),[3,16]));
    assert(isequal(size(radius),[16,1]));
    assert(isequal(size(active),[16,1]));
    assert(sum(active) == expectedActive(mode));
    assert(all(type(telemetry) == 1));
    assert(all(type(map) == 2));
    assert(ground == 0.0 && ceiling == 2.5);
end

% Dynamic entities must move.
[p0,~,~,~,~,~,~,~,~,~,~,~,~] = skyGraphScenarioStage8(0,2,0);
[p1,~,~,~,~,~,~,~,~,~,~,~,~] = skyGraphScenarioStage8(2,2,0);
assert(norm(p1(:,1)-p0(:,1)) > 0.01);

% Scheduled phase/mode checks.
queryTime = [2,6,10,14,18,22,25,29];
expectedAmbient = [0,1,0,0,0,0,0,0];
expectedFault = [0,0,1,2,3,4,5,0];
expectedPhase = 0:7;
for k = 1:length(queryTime)
    [~,~,~,~,~,~,~,~,~,~,ambient,fault,phase] = ...
        skyGraphScenarioStage8(queryTime(k),5,1);
    assert(ambient == expectedAmbient(k));
    assert(fault == expectedFault(k));
    assert(phase == expectedPhase(k));
end

% Diagonal blind-entry scene: unknown ID 399 should not be returned by the
% paired horizontal sensors at level yaw zero.
xTrue = zeros(16,1);
xTrue(3) = 1.0;
[pos,~,rad,id,type,active,~,~,ground,ceiling] = ...
    skyGraphScenarioStage8(0,3,0);
[~,~,~,hitId] = pairedToFGeometryStage8( ...
    xTrue,pos,rad,id,type,active,ground,ceiling);
assert(~any(hitId(1:8) == 399));

% Run dense scene through the complete realistic packet channel.
clear realisticScenarioToFStage8
Ts = 0.05;
time = (0:Ts:30).';
phaseLog = zeros(length(time),1);
faultLog = zeros(length(time),1);
packetAge = zeros(length(time),1);
dropCount = zeros(length(time),1);
activeCount = zeros(length(time),1);
for k = 1:length(time)
    [pos,~,rad,id,type,active,~,~,ground,ceiling,ambient,fault,phase] = ...
        skyGraphScenarioStage8(time(k),5,1);
    [~,~,~,~,~,~,~,~,age,drops] = realisticScenarioToFStage8( ...
        xTrue,time(k),pos,rad,id,type,active,ground,ceiling,ambient,fault);
    phaseLog(k) = phase;
    faultLog(k) = fault;
    packetAge(k) = age;
    dropCount(k) = drops;
    activeCount(k) = sum(active);
end
assert(min(phaseLog) == 0 && max(phaseLog) == 7);
assert(packetAge(1) == 0.0);
assert(dropCount(end) > 0);
assert(max(packetAge(time >= 24 & time < 27)) > 2.0);
assert(all(activeCount == 9));

fprintf('\nSTAGE 8 SCENARIO VALIDATION\n');
fprintf('===========================\n');
fprintf('Fixed 16-entity capacity: PASS\n');
fprintf('Five scenario modes: PASS\n');
fprintf('Telemetry/map source flags: PASS\n');
fprintf('Diagonal blind-entry case: PASS\n');
fprintf('Eight scheduled fault phases: PASS\n');
fprintf('Dense-scene active entities: %d\n',activeCount(1));
fprintf('Cumulative packet drops: %.0f\n',dropCount(end));
fprintf('Maximum packet age during freeze: %.2f s\n', ...
    max(packetAge(time >= 24 & time < 27)));
fprintf('RESULT: PASS - Stage 8 scenarios are ready.\n\n');

figure('Name','Stage 8 scheduled faults');
subplot(3,1,1);
stairs(time,phaseLog,'LineWidth',1.3);
grid on;
ylabel('Phase');
title('Scheduled ambient, sensor and packet faults');
subplot(3,1,2);
stairs(time,faultLog,'LineWidth',1.3);
grid on;
ylabel('Fault mode');
subplot(3,1,3);
plot(time,packetAge,'LineWidth',1.3);
grid on;
xlabel('Time (s)');
ylabel('Packet age (s)');
yline(0.15,'--r','Stale');

% Plot dense-scene entity paths.
figure('Name','Stage 8 dense scene');
hold on;
grid on;
axis equal;
colors = [0.2,0.7,1.0;0.55,0.4,0.25;1.0,0.35,0.2];
for entity = 1:16
    trajectory = nan(length(time),3);
    entityType = 0;
    for k = 1:length(time)
        [pos,~,~,~,types,active] = skyGraphScenarioStage8(time(k),5,0);
        if active(entity)
            trajectory(k,:) = pos(:,entity).';
            entityType = types(entity);
        end
    end
    if entityType > 0
        plot3(trajectory(:,1),trajectory(:,2),trajectory(:,3), ...
            'Color',colors(entityType,:),'LineWidth',1.2);
    end
end
xlabel('World X (m)');
ylabel('World Y (m)');
zlabel('World Z (m)');
title('Dense scene: blue cooperative, brown building, red unknown');
