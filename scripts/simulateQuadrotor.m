%% SIMULATEQUADROTOR Test the equations in MATLAB before using Simulink
clear;
clc;
close all;

% Reference format: [X; Y; Z; yaw]
% Position is in meters and yaw is in radians.
reference = [1; 1; 1; deg2rad(30)];

% Initial state: at the origin, stationary, and level.
x0 = zeros(12,1);

% Simulate for 10 seconds. ode45 numerically integrates dx/dt.
odeFunction = @(t,x) quadDynamics(x, flightController(x, reference)); %#ok<NASGU>
options = odeset('MaxStep', 0.01, 'RelTol', 1e-6, 'AbsTol', 1e-8);
[t, state] = ode45(odeFunction, [0 10], x0, options);

% Recalculate motor speeds at every saved time for plotting.
motorSpeed = zeros(length(t),4);
for k = 1:length(t)
    motorSpeed(k,:) = flightController(state(k,:).', reference).';
end

% Show useful final values in the Command Window.
fprintf('Final position [X Y Z] (m): %.4f  %.4f  %.4f\n', ...
    state(end,1), state(end,2), state(end,3));
fprintf('Final attitude [roll pitch yaw] (deg): %.3f  %.3f  %.3f\n', ...
    state(end,7)*180/pi, state(end,8)*180/pi, state(end,9)*180/pi);
fprintf('Final motor speeds (rad/s): %.2f  %.2f  %.2f  %.2f\n', ...
    motorSpeed(end,1), motorSpeed(end,2), motorSpeed(end,3), motorSpeed(end,4));

% Plot position.
figure('Name','Quadrotor position');
plot(t, state(:,1), 'LineWidth', 1.4);
hold on;
plot(t, state(:,2), 'LineWidth', 1.4);
plot(t, state(:,3), 'LineWidth', 1.8);
yline(reference(1), '--');
yline(reference(2), '--');
yline(reference(3), '--');
grid on;
xlabel('Time (s)');
ylabel('Position (m)');
legend('X','Y','Z','X reference','Y reference','Z reference', ...
    'Location','best');
title('World position');

% Plot attitude.
figure('Name','Quadrotor attitude');
plot(t, state(:,7:9)*180/pi, 'LineWidth', 1.4);
grid on;
xlabel('Time (s)');
ylabel('Angle (degrees)');
legend('Roll','Pitch','Yaw','Location','best');
title('Euler attitude');

% Plot motor speeds.
figure('Name','Motor speeds');
plot(t, motorSpeed, 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('Motor speed (rad/s)');
legend('Motor 1: front','Motor 2: left','Motor 3: rear', ...
    'Motor 4: right','Location','best');
title('Motor commands');
