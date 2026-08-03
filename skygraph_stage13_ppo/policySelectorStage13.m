function [selectedNominal,policySource] = policySelectorStage13( ...
    waypointNominal,normalizedPPOAction,policyMode)
%POLICYSELECTORSTAGE13 Select waypoint baseline or normalized PPO action.

%#codegen

if policyMode(1)>=0.5
    action=min(max(normalizedPPOAction,-1.0),1.0);
    selectedNominal=action.*[2.0;2.0;1.5];
    policySource=1.0;
else
    selectedNominal=waypointNominal;
    policySource=0.0;
end
end
