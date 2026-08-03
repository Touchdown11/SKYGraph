function validateStage15Harness()
%VALIDATESTAGE15HARNESS Check model blocks and required files before batches.

model="quadrotor_stage15_evaluation";
load_system(model);
requiredBlocks=[ ...
 model+"/Stage 8 Scenario Mode"; ...
 model+"/Stage 8 Fault Schedule Enabled"; ...
 model+"/Stage 13 Policy Mode"; ...
 model+"/Stage 15 Safety Mode"; ...
 model+"/Stage 15 Safety Selector"];
for k=1:numel(requiredBlocks)
    if getSimulinkBlockHandle(requiredBlocks(k))<0
        error('Missing or incorrectly named block: %s',requiredBlocks(k));
    end
    fprintf('PASS %s\n',requiredBlocks(k));
end
if isempty(which('ppoAgentStage13.mat')),warning('PPO MAT file not currently on path; configure script loads it by full path.');end
fprintf('Stage 15 harness block check: PASS\n');
end
