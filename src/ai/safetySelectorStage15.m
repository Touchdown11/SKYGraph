function [appliedAcceleration,safetySource] = safetySelectorStage15( ...
    selectedNominal,cbfSafe,safetyMode)
%SAFETYSELECTORSTAGE15 Apply or bypass CBF for simulation-only ablation.
% safetyMode 0 bypass CBF, 1 apply CBF.

%#codegen

if safetyMode(1)>=0.5
    appliedAcceleration=cbfSafe;
    safetySource=1.0;
else
    appliedAcceleration=selectedNominal;
    safetySource=0.0;
end
appliedAcceleration=min(max(appliedAcceleration,[-3;-3;-5]),[3;3;5]);
end
