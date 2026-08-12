function positionError = fcn(xEstimate, reference, controlEnabled)
%#codegen

% Always create a fixed 3x1 output.
positionError = zeros(3,1);

% Index (1) also handles Simulink's one-element vector representation.
if controlEnabled(1)
    positionError(1) = reference(1) - xEstimate(1);
    positionError(2) = reference(2) - xEstimate(2);
    positionError(3) = reference(3) - xEstimate(3);
end
end