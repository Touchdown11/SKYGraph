function configuration=configureMultiEgoStage16(varargin)
%CONFIGUREMULTIEGOSTAGE16 Default configuration for decentralized multi-ego runs.
% All vehicles share a world frame but retain independent local tracking,
% graph, controller and CBF state.
configuration=struct;
configuration.NumberOfEgos=2;
configuration.Duration=30.0;
configuration.SampleTime=0.05;
configuration.EgoRadius=0.22;
configuration.BaseMargin=0.18;
configuration.PolicyMode="waypoint"; % "waypoint" or "ppo" (PPO requires a multi-ego trained agent)
configuration.Scenario="crossing";   % crossing, headon, merge, dense
configuration.TelemetryEnabled=true;
configuration.TelemetryDropoutProbability=0.0;
configuration.RandomSeed=17;
configuration.OutputFile="";
configuration.Visualize=true;
if nargin==1
    supplied=varargin{1};
    names=fieldnames(supplied);
    for k=1:numel(names),configuration.(names{k})=supplied.(names{k});end
elseif nargin>1
    error('MultiEgo:Configuration','Supply zero arguments or one configuration structure.');
end
configuration.PolicyMode=lower(string(configuration.PolicyMode));
configuration.Scenario=lower(string(configuration.Scenario));
if ~any(configuration.Scenario==["crossing","headon","merge","dense"])
    error('MultiEgo:Scenario','Scenario must be crossing, headon, merge, or dense.');
end
validateattributes(configuration.NumberOfEgos,{'numeric'},{'integer','>=',2,'<=',8});
validateattributes(configuration.SampleTime,{'numeric'},{'positive'});
validateattributes(configuration.Duration,{'numeric'},{'positive'});
end
