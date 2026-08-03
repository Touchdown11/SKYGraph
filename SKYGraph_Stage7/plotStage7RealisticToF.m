%% PLOTSTAGE7REALISTICTOF Plot Stage 7 sensor and packet health
if ~exist('out','var')
    error('Run first: out = sim("quadrotor_stage7_realistic_tof");');
end

[t,distance] = readStage7Log(out,'tof7_distance_log',10);
[~,valid] = readStage7Log(out,'tof7_valid_log',10);
[~,status] = readStage7Log(out,'tof7_status_log',10);
[~,ideal] = readStage7Log(out,'tof7_ideal_log',10);
[tDirection,directionDistance] = readStage7Log(out,'tof7_direction_log',6);
[~,directionValid] = readStage7Log(out,'tof7_direction_valid_log',6);
[tAge,packetAge] = readStage7Log(out,'tof7_packet_age_log',1);
[tNew,packetNew] = readStage7Log(out,'tof7_packet_new_log',1);
[tDrop,dropCount] = readStage7Log(out,'tof7_drop_count_log',1);

valid = valid > 0.5;
directionValid = directionValid > 0.5;
maskedDistance = distance;
maskedDistance(~valid) = NaN;
directionDistance(~directionValid) = NaN;

figure('Name','Stage 7 measured versus ideal');
subplot(2,1,1);
plot(t,maskedDistance(:,1:4),'LineWidth',1.0);
grid on;
ylabel('Measured range (m)');
legend('H0','H1','H2','H3','Location','eastoutside');
title('Noisy, offset and quantized ToF ranges');
subplot(2,1,2);
plot(t,distance(:,1)-ideal(:,1),'LineWidth',1.0);
hold on;
plot(t,distance(:,2)-ideal(:,2),'LineWidth',1.0);
grid on;
xlabel('Time (s)');
ylabel('Measured - ideal (m)');
legend('H0 error','H1 error','Location','best');

figure('Name','Stage 7 six direction ranges');
plot(tDirection,directionDistance,'LineWidth',1.3);
grid on;
xlabel('Time (s)');
ylabel('Nearest valid range (m)');
legend('Front','Left','Rear','Right','Up','Down','Location','eastoutside');

figure('Name','Stage 7 validity and status');
subplot(2,1,1);
imagesc(t,0:9,double(valid.'));
axis xy;
ylabel('Sensor ID');
yticks(0:9);
title('Valid mask');
colorbar;
subplot(2,1,2);
imagesc(t,0:9,status.');
axis xy;
xlabel('Time (s)');
ylabel('Sensor ID');
yticks(0:9);
title('Status: 0 valid, 1 no target, 2 too close, 3 dropout, 4 disconnect, 5 stuck');
colorbar;

figure('Name','Stage 7 packet channel');
subplot(3,1,1);
stairs(tNew,packetNew,'LineWidth',1.2);
grid on;
ylabel('New packet');
subplot(3,1,2);
plot(tAge,packetAge,'LineWidth',1.3);
grid on;
ylabel('Age (s)');
yline(0.15,'--r','Stale threshold');
subplot(3,1,3);
stairs(tDrop,dropCount,'LineWidth',1.3);
grid on;
xlabel('Time (s)');
ylabel('Cumulative drops');

function [time,data] = readStage7Log(simulationOutput,name,width)
names = simulationOutput.who;
if any(strcmp(names,name))
    signal = simulationOutput.get(name);
elseif evalin('base',['exist(''' name ''',''var'')'])
    signal = evalin('base',name);
else
    error('Missing log "%s".',name);
end
time = signal.Time(:);
data = squeeze(signal.Data);
if isvector(data)
    data = data(:);
elseif size(data,1) == numel(time)
    % Correct orientation.
elseif size(data,2) == numel(time)
    data = data.';
else
    error('%s has incompatible dimensions.',name);
end
if size(data,2) < width
    error('%s needs %d columns but has %d.',name,width,size(data,2));
end
data = data(:,1:width);
end
