%% PLOTSIMULINKRESULTS Plot the state saved by the Simulink To Workspace block
% First run the model with:
%   out = sim("quadrotor_from_scratch");

if ~exist('out','var')
    error('Variable "out" was not found. Run: out = sim("quadrotor_from_scratch");');
end

% With Single simulation output enabled, the To Workspace result is in out.
outputNames = out.who;
if any(strcmp(outputNames, 'state_log'))
    stateLog = out.get('state_log');
elseif evalin('base', "exist('state_log','var')")
    stateLog = evalin('base','state_log');
else
    error(['state_log was not found. Check the To Workspace block: ', ...
        'Variable name = state_log, Save format = Timeseries.']);
end

time = stateLog.Time;
state = squeeze(stateLog.Data);

% Normally Timeseries stores samples by rows. Correct the orientation if needed.
if size(state,1) ~= numel(time) && size(state,2) == numel(time)
    state = state.';
end

figure('Name','Simulink quadrotor results');
subplot(2,1,1);
plot(time, state(:,1:3), 'LineWidth', 1.4);
grid on;
xlabel('Time (s)');
ylabel('Position (m)');
legend('X','Y','Z','Location','best');
title('Position from Simulink');

subplot(2,1,2);
plot(time, state(:,7:9)*180/pi, 'LineWidth', 1.4);
grid on;
xlabel('Time (s)');
ylabel('Angle (degrees)');
legend('Roll','Pitch','Yaw','Location','best');
title('Attitude from Simulink');
