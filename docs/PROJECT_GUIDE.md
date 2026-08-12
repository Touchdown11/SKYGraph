# SKYGraph — Project Guide

SKYGraph is a MATLAB/Simulink research project that develops an autonomous
quadrotor with a full autonomy stack: time-of-flight perception, multi-object
tracking, a spatial graph map, a Control Barrier Function (CBF) safety shield,
and a Graph-Attention + Proximal-Policy-Optimization (GAT/PPO) threat-aware
policy.

This document explains the repository layout, what was cleaned up, and how to
run the **final visualization** and **evaluation** from within MATLAB.

> The project was originally delivered as 15 "stage" folders with inconsistent
> naming. It has been reorganized into an industry-standard, module-based
> layout. **Function and model names are unchanged**, so all Simulink models
> and cross-file calls keep working.

---

## 1. Directory layout

```
SKYGraph/
├── README.md
├── .gitignore
├── docs/
│   ├── PROJECT_GUIDE.md                 (this file)
│   └── figures/                         (rendered result figures)
├── config/                              # configure*.m — model/path setup per subsystem
├── src/                                 # all MATLAB source functions, grouped by role
│   ├── core/                            # quadrotor dynamics, flight controllers, waypoints, environment
│   ├── estimation/                      # sensor model, state estimator, position error
│   ├── perception/                      # ToF arrays, realistic scenes, ToF geometry
│   ├── tracking/                        # multi-entity tracker
│   ├── mapping/                         # spatial graph builder
│   ├── safety/                          # CBF safety shield
│   └── ai/                              # GAT threat inference + PPO policy + safety selector
├── models/                              # all Simulink .slx models (one per stage)
├── visualization/                       # plot* / animate* scripts (incl. Stage-14 dashboard)
├── scripts/                             # run* / simulate* / train* / evaluate* entry points
├── tests/                               # test* / validate* / diagnose* scripts
├── evaluation/                          # Stage-15 metric extraction
└── data/                                # generated results (.csv / .mat)
```

### Source modules (`src/`)

| Module      | Responsibility                                    | Key functions (deps of the final system) |
|-------------|---------------------------------------------------|------------------------------------------|
| `core`      | Dynamics & flight control, waypoint guidance      | `quadDynamicsStage3`, `waypointTrajectoryStage4`, `attitudeMotorControllerStage11`, `nominalAccelerationStage11` |
| `estimation`| Sensing and state estimation                      | `sensorModelStage3`, `stateEstimatorStage3` |
| `perception`| ToF ranging and scenario generation               | `realisticScenarioToFStage8` |
| `tracking`  | Multi-object tracking                             | `entityTrackerStage9` |
| `mapping`   | Spatial scene graph                               | `graphBuilderStage10` |
| `safety`    | Control Barrier Function shield                   | `cbfSafetyShieldStage11` |
| `ai`        | GAT threat inference + PPO policy                 | `gatThreatInferenceStage12`, `ppoObservationStage13`, `policySelectorStage13`, `safetySelectorStage15` |

---

## 2. Cleanup performed (unwanted folders / code)

The following regenerable or redundant items were identified and removed:

| Removed item                                    | Why                                                        |
|-------------------------------------------------|------------------------------------------------------------|
| `resources/project/**` (XML metadata)           | Auto-generated MATLAB project cache; regenerated on open   |
| `drone_from_scratch.prj`                        | Empty placeholder project file (no content)                |
| `quadrotor_stage13_ppo.slx.original`            | Backup copy of an existing model                           |
| Top-level scratch `.m`/`.slx` clutter           | Moved into `src/core`, `scripts/`, `visualization/`, `models/` (kept as the Stage-1 baseline) |
| Folder names with spaces (`Simulation Stage 8…`) | Non-standard; merged into the new module tree              |

No functional source was deleted — all 94 `.m` files and 13 `.slx` models were
preserved and reorganized. `.gitignore` now excludes `resources/`,
`*.slx.original`, and runtime outputs (`data/*.mat`, `data/*.png`).

---

## 3. Final pipeline (dependency order)

The complete SkyGraph autonomy loop, as configured by
`configureStage15Evaluation`, depends on:

```
core.dynamics/control (stage 3)
   + estimation (stage 3)
   + guidance.waypoints (stage 4)
   + perception.ToF/scenarios (stage 8)
   + tracking.entity tracker (stage 9)
   + mapping.graph builder (stage 10)
   + safety.CBF shield (stage 11)
   + ai.GAT threat inference (stage 12)
   + ai.PPO policy (stage 13)
   → visualization.dashboard (stage 14)
   → evaluation.metrics + experiments (stage 15)
```

---

## 4. Running the final visualization

The **final visualization** is the **Stage-14 SkyGraph Mission Dashboard** — a
multi-panel, 3-D animated view of the simulated mission (true vs. estimated
drone pose, ToF cones, tracked entities, graph + GAT attention, CBF
intervention, and a live status readout).

Prerequisites in MATLAB:
- All folders on the path. Running the entry script from its own folder is
  enough, because it adds the whole project via `genpath`.
- A trained PPO agent (`ppoAgentStage13.mat`) for the Stage-13 model, and the
  Reinforcement Learning Toolbox.

Run the full dashboard (simulates Stage 13, validates logs, then animates):

```matlab
cd <projectRoot>/scripts
out = runStage14Visualization();
```

Or, if `out` from a prior simulation already exists in the workspace:

```matlab
runSkyGraphDashboard           % validates + plays the dashboard on existing `out`
```

Controls during playback: `Space` = pause/resume, `Esc` = stop.

---

## 5. Running the final evaluation (Stage 15)

The **Stage-15 evaluation harness** sweeps policy mode (waypoint vs. PPO) and
safety mode (CBF off vs. on) across scenario and fault conditions (40 runs),
extracts safety/performance metrics, and writes results to `data/`.

```matlab
cd <projectRoot>/scripts
results = runStage15Experiments("full");   % or "quick" for 8 runs
```

Outputs:
- `data/stage15_results.csv`  — per-run metrics
- `data/stage15_summary.csv`  — aggregated by policy × safety mode
- `data/stage15_results.mat`  — raw MATLAB workspace
- Figures from `visualization/plotStage15Results.m`

A rendered example of the final results is in
`docs/figures/stage15_final_results.png`.
