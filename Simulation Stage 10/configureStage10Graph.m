%% CONFIGURESTAGE10GRAPH Configure graph builder at 10 Hz
model = "quadrotor_stage10_graph";
load_system(model);
fastBlocks = [model+"/Stage 8 Scenario Manager"; ...
    model+"/Stage 8 Realistic ToF"; ...
    model+"/Stage 9 Entity Tracker"];
for k = 1:numel(fastBlocks)
    configuration = get_param(fastBlocks(k),"MATLABFunctionConfiguration");
    configuration.UpdateMethod = "Discrete";
    configuration.SampleTime = "0.05";
end
graphConfiguration = get_param(model+"/Stage 10 Graph Builder", ...
    "MATLABFunctionConfiguration");
graphConfiguration.UpdateMethod = "Discrete";
graphConfiguration.SampleTime = "0.10";
set_param(model,'SolverType','Fixed-step','Solver','ode4', ...
    'FixedStep','0.005','StopTime','30');
save_system(model);
fprintf('Configured tracker at 20 Hz and graph builder at 10 Hz.\n');
