%% CONFIGURESTAGE12GAT Configure GAT inference at 10 Hz
model="quadrotor_stage12_gat";
load_system(model);
configuration=get_param(model+"/Stage 12 GAT Threat Encoder", ...
    "MATLABFunctionConfiguration");
configuration.UpdateMethod="Discrete";
configuration.SampleTime="0.10";
set_param(model,'SolverType','Fixed-step','Solver','ode4', ...
    'FixedStep','0.005','StopTime','30');
save_system(model);
fprintf('Configured Stage 12 GAT inference at 10 Hz.\n');
