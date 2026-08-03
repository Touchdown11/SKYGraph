function dz = closedLoopStage2(t, z, reference)
%CLOSEDLOOPSTAGE2 Combined ODE used for the MATLAB-only Stage 2 test.
% z(1:16) contains plant states and z(17:19) contains position-error
% integrals. In Simulink these are implemented by two separate Integrators.

%#codegen

x = z(1:16);
positionIntegral = z(17:19);

omegaCommand = flightControllerStage2(x, reference, positionIntegral);
[windWorld, externalForceWorld, externalTorqueBody] = environmentStage2(t);
plantDerivative = quadDynamicsStage2(x, omegaCommand, windWorld, ...
    externalForceWorld, externalTorqueBody);

% Integrate position error, with clamping anti-windup at the limits.
positionError = reference(1:3) - x(1:3);
integralDerivative = positionError;
integralUpper = [4.0; 4.0; 1.5];
integralLower = -integralUpper;

for axis = 1:3
    if positionIntegral(axis) >= integralUpper(axis) ...
            && positionError(axis) > 0.0
        integralDerivative(axis) = 0.0;
    elseif positionIntegral(axis) <= integralLower(axis) ...
            && positionError(axis) < 0.0
        integralDerivative(axis) = 0.0;
    end
end

dz = zeros(19,1);
dz(1:16) = plantDerivative;
dz(17:19) = integralDerivative;
end
