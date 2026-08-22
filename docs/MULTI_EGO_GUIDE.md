# Stage 16 — Decentralized Multi-Ego Simulation

Stage 16 adds a **new MATLAB fixed-step multi-ego runner** without changing the existing Stage-13/15 single-ego Simulink models. It is deliberately a separate entry point so the established single-ego dashboard and evaluation remain regression-safe.

## What is implemented

- 2–8 independently controlled ego vehicles in a common 3-D world frame.
- A central fixed-capacity world entity table. Controlled egos use permanent IDs `101+`; they are exposed to peers as cooperative type-1 entities. Buildings use `201+`; an optional dense scenario includes a type-3 intruder.
- Independent, ego-centric track tables and SkyGraphs for every ego. Each local graph has exactly one ego node and sees other controlled vehicles as tracks.
- A separate invocation of the existing `cbfSafetyShieldStage11` for every ego at every 0.05-second control update.
- Simultaneous state updates: no ego can use another ego's future state.
- Truth-level pairwise surface separation, collision, local graph, CBF, trajectory, and mission logs.
- A 3-D trajectory/safety plot, CSV summary, MAT output, and validation function.

The first release uses the deterministic waypoint controller as the nominal controller, then applies the existing CBF. `PolicyMode` is retained in the configuration/output schema for later multi-agent PPO attachment, but a single-ego PPO artifact must **not** be treated as a trained multi-ego policy. Attach a PPO only after training it against Stage-16 multi-ego encounters.

## Run

From MATLAB:

```matlab
addpath(genpath(pwd));
[output, metrics] = runMultiEgoStage16();
```

The default run uses two egos on a crossing mission. It writes:

- `data/stage16_multiego_results.mat`
- `data/stage16_multiego_summary.csv`

Run non-interactively or select another scenario:

```matlab
cfg = configureMultiEgoStage16(struct( ...
    'NumberOfEgos', 3, ...
    'Scenario', "dense", ...
    'Duration', 35, ...
    'TelemetryDropoutProbability', 0.15, ...
    'Visualize', false));
[out, metrics] = runMultiEgoStage16(cfg);
report = validateMultiEgoStage16(out);
```

Available scenarios are `"crossing"`, `"headon"`, `"merge"`, and `"dense"`.

## Architecture

```text
world state at k
  -> per-ego permitted source table
  -> local tracker -> local SkyGraph -> waypoint nominal controller -> local CBF
  -> simultaneous state propagation to k+1
```

The world manager never sends an ego its own entity row. Telemetry and map objects are admitted independently; communication dropout can therefore remove a peer track, while a mapped structure remains available.

## Simulink integration boundary

The Stage-16 runner is executable MATLAB simulation infrastructure, not a hand-edited `.slx` file. The existing Stage-9 ToF tracker contains persistent sensor state and must be instantiated once per ego when this is promoted into a new Simulink model. The corresponding model should use a referenced `EgoVehicle` model per vehicle and preserve the timing order above. This avoids cross-ego leakage of tracker, estimator, or sensor state.
