%% SIMULATEQUADROTORSTAGE3 Fixed-step sensor/estimator closed-loop test
clear;
clc;
close all;
clear sensorModelStage3 stateEstimatorStage3

Ts = 0.005;
stopTime = 18.0;
time = (0:Ts:stopTime).';
numberOfSamples = length(time);

m = 1.20;
g = 9.81;
kf = 1.8e-5;
omegaHover = sqrt(m*g/(4*kf));
trueState = [zeros(12,1);omegaHover*ones(4,1)];
positionIntegral = zeros(3,1);
omegaCommand = omegaHover*ones(4,1);

trueStateLog = zeros(numberOfSamples,16);
estimatedStateLog = zeros(numberOfSamples,12);
motorCommandLog = zeros(numberOfSamples,4);
sensorLog = zeros(numberOfSamples,18);
estimatorDiagnosticLog = zeros(numberOfSamples,9);
environmentLog = zeros(numberOfSamples,9);
referenceLog = zeros(numberOfSamples,4);

for k = 1:numberOfSamples
    t = time(k);
    [reference,controlEnabled] = missionReferenceStage3(t);
    [windWorld,externalForceWorld,externalTorqueBody,environmentVector] = ...
        environmentStage3(t);

    % Sensors observe the current plant acceleration produced by the command
    % held from the previous digital-control update.
    trueDerivative = quadDynamicsStage3(trueState,omegaCommand,windWorld, ...
        externalForceWorld,externalTorqueBody);
    [measurement,gpsNew,barometerNew,currentSensorLog] = ...
        sensorModelStage3(trueState,trueDerivative);
    [estimatedState,estimatorDiagnostic] = ...
        stateEstimatorStage3(measurement,gpsNew,barometerNew);

    positionError = positionErrorStage3( ...
        estimatedState,reference,controlEnabled);
    integralUpper = [4.0;4.0;1.5];
    integralLower = -integralUpper;
    for axis = 1:3
        if positionIntegral(axis) >= integralUpper(axis) ...
                && positionError(axis) > 0.0
            positionError(axis) = 0.0;
        elseif positionIntegral(axis) <= integralLower(axis) ...
                && positionError(axis) < 0.0
            positionError(axis) = 0.0;
        end
    end
    positionIntegral = min(max( ...
        positionIntegral+Ts*positionError,integralLower),integralUpper);

    omegaCommand = flightControllerStage3(estimatedState,reference, ...
        positionIntegral,controlEnabled);

    trueStateLog(k,:) = trueState.';
    estimatedStateLog(k,:) = estimatedState.';
    motorCommandLog(k,:) = omegaCommand.';
    sensorLog(k,:) = currentSensorLog.';
    estimatorDiagnosticLog(k,:) = estimatorDiagnostic.';
    environmentLog(k,:) = environmentVector.';
    referenceLog(k,:) = reference.';

    if k < numberOfSamples
        % Fixed-step fourth-order Runge-Kutta plant integration.
        k1 = plantDerivative(t,trueState,omegaCommand);
        k2 = plantDerivative(t+Ts/2.0,trueState+Ts*k1/2.0,omegaCommand);
        k3 = plantDerivative(t+Ts/2.0,trueState+Ts*k2/2.0,omegaCommand);
        k4 = plantDerivative(t+Ts,trueState+Ts*k3,omegaCommand);
        trueState = trueState+Ts*(k1+2.0*k2+2.0*k3+k4)/6.0;
    end
end

positionErrorFinal = estimatedStateLog(end,1:3)-trueStateLog(end,1:3);
attitudeErrorFinal = (estimatedStateLog(end,7:9)-trueStateLog(end,7:9))*180/pi;
fprintf('Final true position (m): %.3f  %.3f  %.3f\n',trueStateLog(end,1:3));
fprintf('Final estimated-minus-true position (m): %.3f  %.3f  %.3f\n', ...
    positionErrorFinal);
fprintf('Final estimated-minus-true attitude (deg): %.2f  %.2f  %.2f\n', ...
    attitudeErrorFinal);
fprintf('Estimated gyro bias (rad/s): %.5f  %.5f  %.5f\n', ...
    estimatorDiagnosticLog(end,1:3));

figure('Name','Stage 3 true and estimated position');
for axis = 1:3
    subplot(3,1,axis);
    plot(time,trueStateLog(:,axis),'LineWidth',1.4);
    hold on;
    plot(time,estimatedStateLog(:,axis),'--','LineWidth',1.2);
    plot(time,referenceLog(:,axis),':','LineWidth',1.2);
    grid on;
    ylabel(['Axis ',num2str(axis),' (m)']);
    if axis == 1
        title('True state is hidden from the controller');
        legend('True','Estimated','Reference','Location','best');
    end
end
xlabel('Time (s)');

figure('Name','Stage 3 estimation errors');
subplot(2,1,1);
plot(time,estimatedStateLog(:,1:3)-trueStateLog(:,1:3),'LineWidth',1.3);
grid on;
ylabel('Position error (m)');
legend('X','Y','Z','Location','best');
title('Estimate minus truth');
subplot(2,1,2);
plot(time,(estimatedStateLog(:,7:9)-trueStateLog(:,7:9))*180/pi, ...
    'LineWidth',1.3);
grid on;
xlabel('Time (s)');
ylabel('Attitude error (deg)');
legend('Roll','Pitch','Yaw','Location','best');

figure('Name','Stage 3 sensor examples');
subplot(3,1,1);
plot(time,sensorLog(:,1:3),'LineWidth',1.0);
grid on;
ylabel('Gyro (rad/s)');
legend('p','q','r','Location','best');
title('Biased and noisy sensor measurements');
subplot(3,1,2);
plot(time,sensorLog(:,10:12),'LineWidth',1.0);
grid on;
ylabel('GPS position (m)');
legend('X','Y','Z','Location','best');
subplot(3,1,3);
plot(time,sensorLog(:,16),'LineWidth',1.0);
grid on;
xlabel('Time (s)');
ylabel('Barometer Z (m)');

figure('Name','Stage 3 motors');
subplot(2,1,1);
plot(time,trueStateLog(:,13:16),'LineWidth',1.1);
grid on;
ylabel('Actual (rad/s)');
legend('Front','Left','Rear','Right','Location','best');
subplot(2,1,2);
plot(time,motorCommandLog-trueStateLog(:,13:16),'LineWidth',1.1);
grid on;
xlabel('Time (s)');
ylabel('Command - actual');

function derivative = plantDerivative(t,state,motorCommand)
[wind,force,torque] = environmentStage3(t);
derivative = quadDynamicsStage3(state,motorCommand,wind,force,torque);
end
