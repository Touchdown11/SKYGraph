%% TESTENTITYTRACKERSTAGE9 Validate tracking and source-based typing
clear;
clc;
close all;
clear realisticScenarioToFStage8 entityTrackerStage9

xEstimate = zeros(12,1);
xEstimate(3) = 1.0;
Ts = 0.05;
time = (0:Ts:30).';
numberOfSamples = length(time);
activeCount = zeros(numberOfSamples,1);
cooperativeCount = zeros(numberOfSamples,1);
buildingCount = zeros(numberOfSamples,1);
unknownCount = zeros(numberOfSamples,1);
minimumTTC = 1000.0*ones(numberOfSamples,1);
trackPositionLog = zeros(numberOfSamples,48);
trackTypeLog = zeros(numberOfSamples,16);
trackSourceLog = zeros(numberOfSamples,16);

for k = 1:numberOfSamples
    [position,velocity,radius,id,~,active,telemetry,map,ground,ceiling, ...
        ambient,fault] = skyGraphScenarioStage8(time(k),5,1);
    [distance,valid,status,~,~,~,~,packetNew,packetAge,~,~,~,~,~] = ...
        realisticScenarioToFStage8(xEstimate,time(k),position,radius,id, ...
        zeros(16,1),active,ground,ceiling,ambient,fault);

    [trackPosition,~,~,~,~,trackId,trackType,trackActive,confidence,age, ...
        sourceMask,~,ttc] = entityTrackerStage9(xEstimate,distance,valid, ...
        status,packetNew,packetAge,position,velocity,radius,id,active, ...
        telemetry,map);

    assert(isequal(size(trackPosition),[3,16]));
    assert(isequal(size(trackActive),[16,1]));
    assert(all(confidence(trackActive) >= 0));
    assert(all(age(trackActive) >= 0));
    assert(all(trackId(trackActive) > 0));

    activeCount(k) = sum(trackActive);
    cooperativeCount(k) = sum(trackActive & trackType == 1);
    buildingCount(k) = sum(trackActive & trackType == 2);
    unknownCount(k) = sum(trackActive & trackType == 3);
    if any(trackActive)
        minimumTTC(k) = min(ttc(trackActive));
    end
    trackPositionLog(k,:) = reshape(trackPosition,48,1).';
    trackTypeLog(k,:) = trackType.';
    trackSourceLog(k,:) = sourceMask.';
end

assert(max(cooperativeCount) >= 3);
assert(max(buildingCount) >= 4);
assert(max(unknownCount) >= 1);
assert(all(trackSourceLog(trackTypeLog == 1) == 1 ...
    | trackSourceLog(trackTypeLog == 1) == 5));
assert(all(trackSourceLog(trackTypeLog == 2) == 2 ...
    | trackSourceLog(trackTypeLog == 2) == 6));
assert(all(trackSourceLog(trackTypeLog == 3) == 4));

% During communication freeze, source-only telemetry/map nodes remain while
% old unassociated ToF tracks are allowed to age out.
preFreezeUnknown = max(unknownCount(time >= 22 & time < 24));
lateFreezeUnknown = min(unknownCount(time >= 26.5 & time < 27));
assert(lateFreezeUnknown <= preFreezeUnknown);

fprintf('\nSTAGE 9 TRACKING VALIDATION\n');
fprintf('===========================\n');
fprintf('Maximum active tracks: %d\n',max(activeCount));
fprintf('Maximum cooperative tracks: %d\n',max(cooperativeCount));
fprintf('Maximum mapped-building tracks: %d\n',max(buildingCount));
fprintf('Maximum unknown ToF tracks: %d\n',max(unknownCount));
fprintf('Unknown tracks before/late freeze: %d / %d\n', ...
    preFreezeUnknown,lateFreezeUnknown);
fprintf('Source-based type masks: PASS\n');
fprintf('RESULT: PASS - Stage 9 entity tracking is ready.\n\n');

figure('Name','Stage 9 track counts');
plot(time,[activeCount,cooperativeCount,buildingCount,unknownCount], ...
    'LineWidth',1.3);
grid on;
xlabel('Time (s)');
ylabel('Track count');
legend('All active','Cooperative','Building','Unknown','Location','best');
title('Fixed-capacity deployed entity table');

figure('Name','Stage 9 minimum TTC');
plot(time,min(minimumTTC,20),'LineWidth',1.3);
grid on;
xlabel('Time (s)');
ylabel('Minimum TTC, clipped at 20 s');
title('Threat kinematics from tracked relative motion');
