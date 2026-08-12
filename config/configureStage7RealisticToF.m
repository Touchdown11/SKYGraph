%% CONFIGURESTAGE7REALISTICTOF Configure realistic ToF block and solver
model = "quadrotor_stage7_realistic_tof";
load_system(model);
configuration = get_param(model+"/Realistic Paired 10-ToF", ...
    "MATLABFunctionConfiguration");
configuration.UpdateMethod = "Discrete";
configuration.SampleTime = "0.05";
set_param(model,'SolverType','Fixed-step','Solver','ode4', ...
    'FixedStep','0.005','StopTime','30');
save_system(model);
fprintf('Configured Stage 7 ToF/channel model at 20 Hz.\n');
