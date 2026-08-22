function report=validateMultiEgoStage16(output)
%VALIDATEMULTIEGOSTAGE16 Verify multi-ego output shape, IDs, and safety logs.
if nargin<1
    config=configureMultiEgoStage16(struct('Duration',4,'Visualize',false));
    output=runMultiEgoSimulationStage16(config);
end
N=output.Configuration.NumberOfEgos; K=numel(output.Time);
assert(isequal(size(output.Position),[3 N K]),'Invalid position log dimensions.');
assert(isequal(size(output.CBFIntervention),[N K]),'Each ego must own a CBF log.');
assert(all(isfinite(output.Position),'all'),'Non-finite ego state found.');
assert(numel(output.MinimumPairwiseSeparation)==K,'Pairwise safety log is incomplete.');
assert(all(output.GraphNodes(:)>=1),'Every local graph must contain its ego node.');
report=struct('Passed',true,'NumberOfEgos',N,'MinimumPairwiseSeparation', ...
    min(output.MinimumPairwiseSeparation),'AnyCollision',any(output.Collision), ...
    'MissionSuccess',output.MissionSuccess);
end
