# SKYGraph Drone Project

Autonomous quadrotor research platform built in **MATLAB / Simulink**. It
combines a quadrotor dynamics/flight-control core with time-of-flight
perception, multi-object tracking, a spatial scene graph, a Control Barrier
Function (CBF) safety shield, and a Graph-Attention + PPO (GAT/PPO) threat-aware
policy, culminating in a full 3-D mission dashboard and an evaluation harness.

## Repository layout (module-based, industry standard)

```
config/         Model & path configuration per subsystem
src/            MATLAB source, grouped by responsibility
  core/         Quadrotor dynamics, controllers, waypoints, environment
  estimation/   Sensor model, state estimator, position error
  perception/   ToF arrays, realistic scenes, ToF geometry
  tracking/     Multi-entity tracker
  mapping/      Spatial graph builder
  safety/       CBF safety shield
  ai/           GAT threat inference + PPO policy + safety selector
models/         All Simulink .slx models
visualization/  plot* / animate* scripts (incl. Stage-14 dashboard)
scripts/        run* / simulate* / train* / evaluate* entry points
tests/          test* / validate* / diagnose* scripts
evaluation/     Stage-15 metric extraction
data/           Generated results (.csv / .mat)
docs/           Guides and rendered result figures
```

## Getting started

1. Open the project in MATLAB and add all folders to the path:
   ```matlab
   addpath(genpath(pwd));
   ```
2. Run the **final visualization** (Stage-14 mission dashboard):
   ```matlab
   cd scripts
   out = runStage14Visualization();
   ```
3. Run the **final evaluation** (Stage-15 40-run sweep):
   ```matlab
   cd scripts
   results = runStage15Experiments("full");
   ```

See [`docs/PROJECT_GUIDE.md`](docs/PROJECT_GUIDE.md) for details, the full
dependency map, the cleanup log, and MATLAB usage notes.

## Requirements

- MATLAB with **Simulink** and the **Reinforcement Learning Toolbox**
- A trained PPO agent (`ppoAgentStage13.mat`) for the Stage-13 model
