%% CONFIGURESTAGE11CBF Configure multi-rate CBF control architecture
model = "quadrotor_stage11_cbf";
load_system(model);

blocks20Hz = [model+"/Stage 8 Scenario Manager"; ...
              model+"/Stage 8 Realistic ToF"; ...
              model+"/Stage 9 Entity Tracker"; ...
              model+"/Stage 11 Nominal Acceleration"; ...
              model+"/Stage 11 CBF Shield"];
for k=1:numel(blocks20Hz)
    configuration=get_param(blocks20Hz(k),"MATLABFunctionConfiguration");
    configuration.UpdateMethod="Discrete";
    configuration.SampleTime="0.05";
end

graphConfiguration=get_param(model+"/Stage 10 Graph Builder", ...
    "MATLABFunctionConfiguration");
graphConfiguration.UpdateMethod="Discrete";
graphConfiguration.SampleTime="0.10";

attitudeConfiguration=get_param(model+"/Stage 11 Attitude and Mixer", ...
    "MATLABFunctionConfiguration");
attitudeConfiguration.UpdateMethod="Discrete";
attitudeConfiguration.SampleTime="0.005";

set_param(model,'SolverType','Fixed-step','Solver','ode4', ...
    'FixedStep','0.005','StopTime','30');
save_system(model);
fprintf('Configured CBF at 20 Hz, graph at 10 Hz and attitude loop at 200 Hz.\n');
