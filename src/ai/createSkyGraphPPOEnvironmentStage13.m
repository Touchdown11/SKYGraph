function [environment,observationInfo,actionInfo] = ...
    createSkyGraphPPOEnvironmentStage13()
%CREATESKYGRAPHPPOENVIRONMENTSTAGE13 Function environment for safe PPO.

observationInfo=rlNumericSpec([42 1]);
observationInfo.Name="SkyGraph observation";
observationInfo.Description="goal, ego velocity, previous safe action, top-3 GAT entities";
actionInfo=rlNumericSpec([3 1],LowerLimit=-ones(3,1),UpperLimit=ones(3,1));
actionInfo.Name="Normalized nominal acceleration";
environment=rlFunctionEnv(observationInfo,actionInfo, ...
    @stepSkyGraphPPOStage13,@resetSkyGraphPPOStage13);
end
