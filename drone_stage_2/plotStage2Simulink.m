%% PLOTSTAGE2SIMULINK Plot Stage 2 signals saved by Simulink
% Run the model first:
%   out = sim("quadrotor_stage2");

if ~exist('out','var')
    error('Run the model first: out = sim("quadrotor_stage2");');
end

stateTs = getStage2Log(out,'state2_log');
commandTs = getStage2Log(out,'motorcmd2_log');
environmentTs = getStage2Log(out,'env2_log');
integralTs = getStage2Log(out,'integral2_log');

% Keep each signal's own time vector. Variable-step simulation can save
% different numbers of samples for different signals.
[tState,state] = timeAndMatrix(stateTs,'state2_log');
[tCommand,motorCommand] = timeAndMatrix(commandTs,'motorcmd2_log');
[tEnvironment,environment] = timeAndMatrix(environmentTs,'env2_log');
[tIntegral,positionIntegral] = timeAndMatrix(integralTs,'integral2_log');

% Check widths before plotting so a logging connection error is clear.
if size(state,2) < 16
    error('state2_log must contain 16 columns, but it contains %d.',size(state,2));
end
if size(motorCommand,2) < 4
    error('motorcmd2_log must contain 4 columns, but it contains %d.',size(motorCommand,2));
end
if size(environment,2) < 9
    error('env2_log must contain 9 columns, but it contains %d.',size(environment,2));
end
if size(positionIntegral,2) < 3
    error('integral2_log must contain 3 columns, but it contains %d.',size(positionIntegral,2));
end

figure('Name','Stage 2 Simulink position');
subplot(2,1,1);
plot(tState,state(:,1:3),'LineWidth',1.4);
grid on;
ylabel('Position (m)');
legend('X','Y','Z','Location','best');
title('Position under persistent wind and disturbances');
subplot(2,1,2);
plot(tIntegral,positionIntegral(:,1:3),'LineWidth',1.3);
grid on;
xlabel('Time (s)');
ylabel('Integral (m s)');
legend('X integral','Y integral','Z integral','Location','best');

figure('Name','Stage 2 Simulink attitude');
plot(tState,state(:,7:9)*180/pi,'LineWidth',1.4);
grid on;
xlabel('Time (s)');
ylabel('Angle (degrees)');
legend('Roll','Pitch','Yaw','Location','best');
title('Attitude response');

figure('Name','Stage 2 Simulink motors');
subplot(2,1,1);
plot(tState,state(:,13:16),'LineWidth',1.2);
grid on;
ylabel('Actual speed (rad/s)');
legend('Front','Left','Rear','Right','Location','best');
title('Actual motor states');

% Interpolate actual speeds onto command-log times when sample counts differ.
[tStateUnique,uniqueIndices] = unique(tState,'stable');
actualMotorUnique = state(uniqueIndices,13:16);
actualMotorAtCommandTime = interp1(tStateUnique,actualMotorUnique, ...
    tCommand,'linear','extrap');

subplot(2,1,2);
plot(tCommand,motorCommand(:,1:4)-actualMotorAtCommandTime,'LineWidth',1.2);
grid on;
xlabel('Time (s)');
ylabel('Command - actual (rad/s)');
legend('Front','Left','Rear','Right','Location','best');
title('Motor lag error');

figure('Name','Stage 2 Simulink environment');
subplot(3,1,1);
plot(tEnvironment,environment(:,1:3),'LineWidth',1.3);
grid on;
ylabel('Wind (m/s)');
legend('X','Y','Z','Location','best');
subplot(3,1,2);
plot(tEnvironment,environment(:,4:6),'LineWidth',1.3);
grid on;
ylabel('Force (N)');
legend('X','Y','Z','Location','best');
subplot(3,1,3);
plot(tEnvironment,environment(:,7:9),'LineWidth',1.3);
grid on;
xlabel('Time (s)');
ylabel('Torque (N m)');
legend('Roll','Pitch','Yaw','Location','best');

function signal = getStage2Log(simulationOutput,name)
outputNames = simulationOutput.who;
if any(strcmp(outputNames,name))
    signal = simulationOutput.get(name);
elseif evalin('base',['exist(''' name ''',''var'')'])
    signal = evalin('base',name);
else
    error('Missing log "%s". Check its To Workspace block.',name);
end
end

function [time,data] = timeAndMatrix(signal,signalName)
time = signal.Time(:);
data = squeeze(signal.Data);

% Convert all common Timeseries layouts to samples-by-signals.
if isvector(data)
    data = data(:);
elseif size(data,1) == numel(time)
    % Already N-by-width; no change required.
elseif size(data,2) == numel(time)
    data = data.';
else
    error(['%s has %d time samples, but its squeezed data size is %s. ', ...
           'Set its To Workspace Save format to Timeseries.'], ...
           signalName,numel(time),mat2str(size(data)));
end
end
