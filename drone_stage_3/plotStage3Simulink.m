%% PLOTSTAGE3SIMULINK Plot Stage 3 Simulink logs
% First run: out = sim("quadrotor_stage3");

if ~exist('out','var')
    error('Run first: out = sim("quadrotor_stage3");');
end

[tTrue,xTrue] = readLog(out,'true_state3_log',16);
[tEst,xEst] = readLog(out,'estimated_state3_log',12);
[tSensor,sensors] = readLog(out,'sensor3_log',18);
[tCommand,motorCommand] = readLog(out,'motorcmd3_log',4);
[tEnvironment,environment] = readLog(out,'environment3_log',9);
[tReference,reference] = readLog(out,'reference3_log',4);
[tEstimator,estimatorDiagnostic] = readLog(out,'estimator3_log',9);

trueAtEstimate = interpolateSignal(tTrue,xTrue(:,1:12),tEst);
actualMotorAtCommand = interpolateSignal(tTrue,xTrue(:,13:16),tCommand);

figure('Name','Stage 3 position: true versus estimated');
for axis = 1:3
    subplot(3,1,axis);
    plot(tTrue,xTrue(:,axis),'LineWidth',1.4);
    hold on;
    plot(tEst,xEst(:,axis),'--','LineWidth',1.2);
    plot(tReference,reference(:,axis),':','LineWidth',1.2);
    grid on;
    ylabel(['Axis ',num2str(axis),' (m)']);
    if axis == 1
        title('Controller receives estimated state only');
        legend('True','Estimated','Reference','Location','best');
    end
end
xlabel('Time (s)');

figure('Name','Stage 3 estimator error');
subplot(2,1,1);
plot(tEst,xEst(:,1:3)-trueAtEstimate(:,1:3),'LineWidth',1.3);
grid on;
ylabel('Position error (m)');
legend('X','Y','Z','Location','best');
title('Estimate minus truth');
subplot(2,1,2);
plot(tEst,(xEst(:,7:9)-trueAtEstimate(:,7:9))*180/pi,'LineWidth',1.3);
grid on;
xlabel('Time (s)');
ylabel('Attitude error (deg)');
legend('Roll','Pitch','Yaw','Location','best');

figure('Name','Stage 3 sensor measurements');
subplot(3,1,1);
plot(tSensor,sensors(:,1:3),'LineWidth',1.0);
grid on;
ylabel('Gyro (rad/s)');
legend('p','q','r','Location','best');
title('Noisy, biased, multi-rate measurements');
subplot(3,1,2);
plot(tSensor,sensors(:,10:12),'LineWidth',1.0);
grid on;
ylabel('GPS position (m)');
legend('X','Y','Z','Location','best');
subplot(3,1,3);
plot(tSensor,sensors(:,16),'LineWidth',1.0);
grid on;
xlabel('Time (s)');
ylabel('Barometer Z (m)');

figure('Name','Stage 3 gyro-bias estimate');
plot(tEstimator,estimatorDiagnostic(:,1:3),'LineWidth',1.3);
grid on;
xlabel('Time (s)');
ylabel('Bias estimate (rad/s)');
legend('X','Y','Z','Location','best');
title('One-second stationary gyroscope calibration');

figure('Name','Stage 3 motors');
subplot(2,1,1);
plot(tTrue,xTrue(:,13:16),'LineWidth',1.1);
grid on;
ylabel('Actual speed (rad/s)');
legend('Front','Left','Rear','Right','Location','best');
subplot(2,1,2);
plot(tCommand,motorCommand-actualMotorAtCommand,'LineWidth',1.1);
grid on;
xlabel('Time (s)');
ylabel('Command - actual');

figure('Name','Stage 3 environment');
subplot(3,1,1);
plot(tEnvironment,environment(:,1:3),'LineWidth',1.2);
grid on;
ylabel('Wind (m/s)');
legend('X','Y','Z','Location','best');
subplot(3,1,2);
plot(tEnvironment,environment(:,4:6),'LineWidth',1.2);
grid on;
ylabel('Force (N)');
legend('X','Y','Z','Location','best');
subplot(3,1,3);
plot(tEnvironment,environment(:,7:9),'LineWidth',1.2);
grid on;
xlabel('Time (s)');
ylabel('Torque (N m)');
legend('Roll','Pitch','Yaw','Location','best');

function [time,data] = readLog(simulationOutput,name,requiredWidth)
outputNames = simulationOutput.who;
if any(strcmp(outputNames,name))
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
    % Correct orientation already.
elseif size(data,2) == numel(time)
    data = data.';
else
    error('%s time/data sizes are incompatible.',name);
end
if size(data,2) < requiredWidth
    error('%s needs %d columns but has %d.',name,requiredWidth,size(data,2));
end
data = data(:,1:requiredWidth);
end

function interpolated = interpolateSignal(time,data,newTime)
[uniqueTime,indices] = unique(time,'stable');
interpolated = interp1(uniqueTime,data(indices,:),newTime,'linear','extrap');
end
