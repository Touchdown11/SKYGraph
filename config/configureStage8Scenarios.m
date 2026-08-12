%% CONFIGURESTAGE8SCENARIOS Configure scenario and sensor blocks
model = "quadrotor_stage8_scenarios";
load_system(model);
blocks = [model+"/Stage 8 Scenario Manager"; ...
    model+"/Stage 8 Realistic ToF"];
for k = 1:numel(blocks)
    configuration = get_param(blocks(k),"MATLABFunctionConfiguration");
    configuration.UpdateMethod = "Discrete";
    configuration.SampleTime = "0.05";
end
set_param(model,'SolverType','Fixed-step','Solver','ode4', ...
    'FixedStep','0.005','StopTime','30');
save_system(model);
fprintf('Configured Stage 8 scenario and ToF blocks at 20 Hz.\n');
