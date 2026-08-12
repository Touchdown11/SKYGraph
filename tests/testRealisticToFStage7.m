%% TESTREALISTICTOFSTAGE7 Validate noise, faults, latency and packet loss
clear;
clc;
close all;

xTrue = zeros(16,1);
xTrue(3) = 1.0;
Ts = 0.05;
time = (0:Ts:10).';

% Normal indoor run.
clear realisticPairedToFStage7
numberOfSamples = length(time);
distanceNormal = zeros(numberOfSamples,10);
idealNormal = zeros(numberOfSamples,10);
validNormal = false(numberOfSamples,10);
packetNewNormal = false(numberOfSamples,1);
packetAgeNormal = zeros(numberOfSamples,1);
dropNormal = zeros(numberOfSamples,1);
for k = 1:numberOfSamples
    [distanceSample,validSample,~,~,~,~,~,packetNewSample, ...
        packetAgeSample,dropSample,idealSample] = ...
        realisticPairedToFStage7(xTrue,time(k),0.0,0.0);
    distanceNormal(k,:) = distanceSample.';
    validNormal(k,:) = validSample.';
    packetNewNormal(k) = packetNewSample;
    packetAgeNormal(k) = packetAgeSample;
    dropNormal(k) = dropSample;
    idealNormal(k,:) = idealSample.';
end
normalMask = validNormal & isfinite(distanceNormal) & isfinite(idealNormal);
normalError = distanceNormal(normalMask)-idealNormal(normalMask);
normalRms = sqrt(mean(normalError.^2));
assert(normalRms > 0.001 && normalRms < 0.10);
assert(dropNormal(end) == 0);
assert(all(packetNewNormal(2:end)));
assert(max(abs(packetAgeNormal(2:end)-Ts)) < 1.0e-10);

% Bright/low-reflectivity stress should create more error/dropout.
clear realisticPairedToFStage7
validBright = false(numberOfSamples,10);
distanceBright = zeros(numberOfSamples,10);
idealBright = zeros(numberOfSamples,10);
for k = 1:numberOfSamples
    [distanceSample,validSample,~,~,~,~,~,~,~,~,idealSample] = ...
        realisticPairedToFStage7(xTrue,time(k),1.0,0.0);
    distanceBright(k,:) = distanceSample.';
    validBright(k,:) = validSample.';
    idealBright(k,:) = idealSample.';
end
brightMask = validBright & isfinite(distanceBright) & isfinite(idealBright);
brightError = distanceBright(brightMask)-idealBright(brightMask);
brightRms = sqrt(mean(brightError.^2));
assert(brightRms > normalRms);
assert(sum(~validBright(:)) > sum(~validNormal(:)));

% H2 disconnected.
clear realisticPairedToFStage7
realisticPairedToFStage7(xTrue,0.0,0.0,2.0);
[~,validDisconnected,statusDisconnected] = ...
    realisticPairedToFStage7(xTrue,Ts,0.0,2.0);
assert(~validDisconnected(3) && statusDisconnected(3) == 4.0);

% H0 stuck.
clear realisticPairedToFStage7
realisticPairedToFStage7(xTrue,0.0,0.0,3.0);
[distanceStuck,validStuck,statusStuck] = ...
    realisticPairedToFStage7(xTrue,Ts,0.0,3.0);
assert(validStuck(1) && abs(distanceStuck(1)-0.450) < 1.0e-12);
assert(statusStuck(1) == 5.0);

% Random packet loss.
clear realisticPairedToFStage7
packetNewLoss = false(numberOfSamples,1);
dropLoss = zeros(numberOfSamples,1);
for k = 1:numberOfSamples
    [~,~,~,~,~,~,~,packetNewLoss(k),~,dropLoss(k)] = ...
        realisticPairedToFStage7(xTrue,time(k),0.0,4.0);
end
assert(dropLoss(end) > 0);
assert(any(~packetNewLoss(3:end)));

% Communication freeze after 5 seconds must make packet age grow.
clear realisticPairedToFStage7
freezeTime = (0:Ts:8).';
freezeAge = zeros(length(freezeTime),1);
for k = 1:length(freezeTime)
    [~,~,~,~,~,~,~,~,freezeAge(k)] = ...
        realisticPairedToFStage7(xTrue,freezeTime(k),0.0,5.0);
end
assert(freezeAge(end) > 2.5);

fprintf('\nSTAGE 7 REALISTIC TOF VALIDATION\n');
fprintf('================================\n');
fprintf('Normal indoor RMS error: %.4f m\n',normalRms);
fprintf('Bright-mode RMS error:   %.4f m\n',brightRms);
fprintf('Normal packet latency:   %.3f s\n',Ts);
fprintf('Random-loss dropped packets: %.0f\n',dropLoss(end));
fprintf('Freeze final packet age: %.3f s\n',freezeAge(end));
fprintf('H2 disconnect handling: PASS\n');
fprintf('H0 stuck-value handling: PASS\n');
fprintf('RESULT: PASS - Stage 7 sensor/channel faults are ready.\n\n');

figure('Name','Stage 7 noise comparison');
subplot(2,1,1);
plot(time,distanceNormal(:,1),'LineWidth',1.0);
hold on;
plot(time,idealNormal(:,1),'--','LineWidth',1.2);
grid on;
ylabel('H0 range (m)');
legend('Normal measured','Ideal delayed','Location','best');
subplot(2,1,2);
plot(time,distanceBright(:,1),'LineWidth',1.0);
hold on;
plot(time,idealBright(:,1),'--','LineWidth',1.2);
grid on;
xlabel('Time (s)');
ylabel('H0 range (m)');
legend('Bright measured','Ideal delayed','Location','best');

figure('Name','Stage 7 network faults');
subplot(2,1,1);
stairs(time,packetNewLoss,'LineWidth',1.2);
grid on;
ylabel('New packet');
title('Thirty-percent random packet-loss mode');
subplot(2,1,2);
plot(freezeTime,freezeAge,'LineWidth',1.4);
grid on;
xlabel('Time (s)');
ylabel('Packet age (s)');
title('Communication freeze after five seconds');
