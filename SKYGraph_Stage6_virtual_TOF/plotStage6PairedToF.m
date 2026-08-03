%% PLOTSTAGE6PAIREDTOF Plot clean paired ToF Simulink logs
if ~exist('out','var')
    error('Run first: out = sim("quadrotor_stage6_paired_tof");');
end

[tSensor,distance] = readPairedLog(out,'paired_distance6_log',10);
[~,valid] = readPairedLog(out,'paired_valid6_log',10);
[tDirection,directionDistance] = readPairedLog(out,'direction_distance6_log',6);
[~,directionValid] = readPairedLog(out,'direction_valid6_log',6);
[tTrue,xTrue] = readPairedLog(out,'true_state6_log',16);

valid = valid > 0.5;
directionValid = directionValid > 0.5;
distance(~valid) = NaN;
directionDistance(~directionValid) = NaN;

figure('Name','Paired ToF sensor ranges');
plot(tSensor,distance,'LineWidth',1.1);
grid on;
xlabel('Time (s)');
ylabel('Range (m)');
legend('H0 front-right','H1 front-left','H2 left-front','H3 left-rear', ...
    'H4 rear-left','H5 rear-right','H6 right-rear','H7 right-front', ...
    'V0 up','V1 down','Location','eastoutside');

figure('Name','Six coarse detection directions');
plot(tDirection,directionDistance,'LineWidth',1.4);
grid on;
xlabel('Time (s)');
ylabel('Nearest valid range (m)');
legend('Front','Left','Rear','Right','Up','Down','Location','eastoutside');
title('Nearest sensor in each paired direction');

figure('Name','Paired sensor valid mask');
imagesc(tSensor,0:9,double(valid.'));
axis xy;
xlabel('Time (s)');
ylabel('Sensor ID');
yticks(0:9);
colorbar;
title('Ten-sensor valid mask');

figure('Name','Drone context');
subplot(2,1,1);
plot(tTrue,xTrue(:,1:3),'LineWidth',1.3);
grid on;
ylabel('Position (m)');
legend('X','Y','Z','Location','best');
subplot(2,1,2);
plot(tTrue,xTrue(:,7:9)*180/pi,'LineWidth',1.3);
grid on;
xlabel('Time (s)');
ylabel('Attitude (deg)');
legend('Roll','Pitch','Yaw','Location','best');

function [time,data] = readPairedLog(simulationOutput,name,width)
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
