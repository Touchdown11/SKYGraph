%% SIMULATEQUADROTORSTAGE2 Test motor lag, wind, disturbances, and PID control
clear;
clc;
close all;

reference = [0; 0; 1; 0];  % desired [X; Y; Z; yaw]

% Start level and stationary, but with already-armed motors at hover speed.
m = 1.20;
g = 9.81;
kf = 1.8e-5;
omegaHover = sqrt(m*g/(4*kf));
plantInitialState = [zeros(12,1); omegaHover*ones(4,1)];
positionIntegralInitialState = zeros(3,1);
z0 = [plantInitialState; positionIntegralInitialState];

options = odeset('MaxStep',0.005,'RelTol',1e-6,'AbsTol',1e-8);
[t, combinedState] = ode45( ...
    @(time,z) closedLoopStage2(time,z,reference), [0 15], z0, options);

plantState = combinedState(:,1:16);
positionIntegral = combinedState(:,17:19);
numberOfSamples = length(t);
motorCommand = zeros(numberOfSamples,4);
environmentLog = zeros(numberOfSamples,9);

for k = 1:numberOfSamples
    motorCommand(k,:) = flightControllerStage2( ...
        plantState(k,:).', reference, positionIntegral(k,:).').';
    [~,~,~,environmentVector] = environmentStage2(t(k));
    environmentLog(k,:) = environmentVector.';
end

fprintf('Hover speed: %.2f rad/s\n', omegaHover);
fprintf('Final position [X Y Z] (m): %.3f  %.3f  %.3f\n', ...
    plantState(end,1), plantState(end,2), plantState(end,3));
fprintf('Final attitude [roll pitch yaw] (deg): %.2f  %.2f  %.2f\n', ...
    plantState(end,7)*180/pi, plantState(end,8)*180/pi, ...
    plantState(end,9)*180/pi);
fprintf('Final position integrals (m*s): %.3f  %.3f  %.3f\n', ...
    positionIntegral(end,1), positionIntegral(end,2), ...
    positionIntegral(end,3));

% Position plot.
figure('Name','Stage 2 position');
plot(t,plantState(:,1:3),'LineWidth',1.5);
hold on;
yline(reference(1),'--');
yline(reference(2),'--');
yline(reference(3),'--');
grid on;
xlabel('Time (s)');
ylabel('Position (m)');
legend('X','Y','Z','X reference','Y reference','Z reference', ...
    'Location','best');
title('Position under wind and disturbances');

% Attitude plot.
figure('Name','Stage 2 attitude');
plot(t,plantState(:,7:9)*180/pi,'LineWidth',1.4);
grid on;
xlabel('Time (s)');
ylabel('Angle (degrees)');
legend('Roll','Pitch','Yaw','Location','best');
title('Attitude response');

% Actual motors and motor-lag error.
figure('Name','Stage 2 motors');
subplot(2,1,1);
plot(t,plantState(:,13:16),'LineWidth',1.2);
grid on;
ylabel('Actual speed (rad/s)');
legend('Front','Left','Rear','Right','Location','best');
title('Actual motor speeds are now dynamic states');
subplot(2,1,2);
plot(t,motorCommand-plantState(:,13:16),'LineWidth',1.2);
grid on;
xlabel('Time (s)');
ylabel('Command - actual (rad/s)');
legend('Front','Left','Rear','Right','Location','best');
title('Motor lag error');

% Environment plot.
figure('Name','Stage 2 environment');
subplot(3,1,1);
plot(t,environmentLog(:,1:3),'LineWidth',1.3);
grid on;
ylabel('Wind (m/s)');
legend('X','Y','Z','Location','best');
title('Applied environment');
subplot(3,1,2);
plot(t,environmentLog(:,4:6),'LineWidth',1.3);
grid on;
ylabel('Force (N)');
legend('X','Y','Z','Location','best');
subplot(3,1,3);
plot(t,environmentLog(:,7:9),'LineWidth',1.3);
grid on;
xlabel('Time (s)');
ylabel('Torque (N m)');
legend('Roll','Pitch','Yaw','Location','best');
