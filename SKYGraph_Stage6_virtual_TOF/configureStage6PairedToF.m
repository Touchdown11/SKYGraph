%% CONFIGURESTAGE6PAIREDTOF Configure clean paired ToF model
model = "quadrotor_stage6_paired_tof";
load_system(model);
configuration = get_param(model+"/Paired 10-ToF Array", ...
    "MATLABFunctionConfiguration");
configuration.UpdateMethod = "Discrete";
configuration.SampleTime = "0.05";
set_param(model,'SolverType','Fixed-step','Solver','ode4', ...
    'FixedStep','0.005','StopTime','30');
save_system(model);
fprintf('Configured paired ToF at 20 Hz and plant at 200 Hz.\n');
