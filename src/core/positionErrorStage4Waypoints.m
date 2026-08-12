function positionError = positionErrorStage4Waypoints( ...
    xEstimate, trajectoryReference, controlEnabled)
%POSITIONERRORSTAGE4WAYPOINTS Fixed-size integral tracking error.

%#codegen

positionError = zeros(3,1);
if controlEnabled(1)
    positionError(1) = trajectoryReference(1)-xEstimate(1);
    positionError(2) = trajectoryReference(2)-xEstimate(2);
    positionError(3) = trajectoryReference(3)-xEstimate(3);
end
end
