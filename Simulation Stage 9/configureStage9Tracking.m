%% CONFIGURESTAGE9TRACKING Configure scenario, ToF and tracker at 20 Hz
model = "quadrotor_stage9_tracking";
load_system(model);
blocks = [model+"/Stage 8 Scenario Manager"; ...
    model+"/Stage 8 Realistic ToF"; ...
    model+"/Stage 9 Entity Tracker"];
for k = 1:numel(blocks)
    configuration = get_param(blocks(k),"MATLABFunctionConfiguration");
    configuration.UpdateMethod = "Discrete";
    configuration.SampleTime = "0.05";
end
set_param(model,'SolverType','Fixed-step','Solver','ode4', ...
    'FixedStep','0.005','StopTime','30');
save_system(model);
fprintf('Configured Stage 9 scenario, ToF and tracker at 20 Hz.\n');
