function agent = createPPOAgentStage13(observationInfo,actionInfo)
%CREATEPPOAGENTSTAGE13 Initialize continuous-action PPO agent.

initialization=rlAgentInitializationOptions(NumHiddenUnit=128);
agent=rlPPOAgent(observationInfo,actionInfo,initialization);
options=agent.AgentOptions;
options.SampleTime=0.10;
options.DiscountFactor=0.99;
options.ExperienceHorizon=256;
options.MiniBatchSize=64;
options.NumEpoch=3;
options.ClipFactor=0.20;
options.EntropyLossWeight=0.01;
options.ActorOptimizerOptions.LearnRate=3e-4;
options.CriticOptimizerOptions.LearnRate=1e-3;
options.ActorOptimizerOptions.GradientThreshold=1;
options.CriticOptimizerOptions.GradientThreshold=1;
agent.AgentOptions=options;
end
