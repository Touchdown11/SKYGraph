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
> layout, and every file **not required to run the final Stage-14
> visualization and Stage-15 evaluation** has been removed. **Function and
> model names are unchanged**, so all remaining Simulink models and cross-file
> calls keep working.

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

Two cleanup passes were made.

**Pass 1 — regenerable clutter removed:**

| Removed item                                    | Why                                                        |
|-------------------------------------------------|------------------------------------------------------------|
| `resources/project/**` (XML metadata)           | Auto-generated MATLAB project cache; regenerated on open   |
| `drone_from_scratch.prj`                        | Empty placeholder project file (no content)                |
| `quadrotor_stage13_ppo.slx.original`            | Backup copy of an existing model                           |
| Folder names with spaces (`Simulation Stage 8…`) | Non-standard; merged into the new module tree              |

**Pass 2 — stages not needed to run Stage 14 & 15 removed:**

The repository contained the full development history of 15 incremental
stages. Only the modules that the **final Stage-14 visualization** and
**Stage-15 evaluation** actually depend on were retained (stages 3 dynamics /
estimation, 4 waypoint guidance, 8 scenarios/ToF, 9 tracking, 10 graph, 11 CBF,
12 GAT, 13 PPO, plus the training scripts that produce the required agent and
weights). Everything else was removed:

| Removed                                                          | Why                                             |
|------------------------------------------------------------------|-------------------------------------------------|
| Stage 1 baseline + Stage 2 (models, `flightController`, `quadDynamics`, sim scripts) | Superseded by Stage 3 dynamics/control |
| Stage 4 / Stage 5 legacy visualization (`animateDroneStage4`, waypoint-viz, sim scripts) | Superseded by Stage 14 dashboard |
| Stage 6 & Stage 7 ToF models (`pairedScene`, `pairedToFArray`, `realisticPairedToF`) | Superseded by Stage 8 scenarios/ToF |
| Intermediate `configureStage*` / `plotStage*` / `testStage*` for stages 8–13 | Not invoked by the final run flows |
| Intermediate `.slx` models (stage 2–12)                        | Only the Stage 13 & 15 models are simulated |
| Legacy entry scripts (`simulateQuadrotor*`, `runStage4/5*`)      | Replaced by `runStage14Visualization` / `runStage15Experiments` |

The retained set is a complete, runnable final pipeline — every remaining
source function is a transitive dependency of the two final models or of the
training scripts that generate their runtime artifacts
(`ppoAgentStage13.mat`, `gatThreatWeightsStage12.mat`). `.gitignore` excludes
`resources/`, `*.slx.original`, and runtime outputs (`data/*.mat`, `data/*.png`).

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

## 4. First-time setup (generate trained artifacts)

The final pipeline needs two **trained artifacts** that are generated at
runtime and are **not stored in the repository** (they are `.mat`, gitignored):

1. `gatThreatWeightsStage12.mat` — GAT threat encoder weights (Stage 12)
2. `ppoAgentStage13.mat` — PPO navigation policy (Stage 13)

If they are missing you will get an error like
`ppoAgentStage13.mat was not found`. Generate them once before the first run:

```matlab
addpath(genpath(pwd));
[weightsFile, agentFile] = setupSkyGraph();
```

`setupSkyGraph` trains Stage 12 first (Stage 13 depends on it) and skips any
artifact that already exists. Requires MATLAB with the **Reinforcement Learning
Toolbox** and **Deep Learning Toolbox**.

## 5. Running the final visualization (Stage 14 dashboard)

The **final visualization** is the **Stage-14 SkyGraph Mission Dashboard** — a
multi-panel, 3-D animated view of the simulated mission (true vs. estimated
drone pose, ToF cones, tracked entities, graph + GAT attention, CBF
intervention, and a live status readout).

Prerequisites in MATLAB:
- Trained artifacts generated via `setupSkyGraph` (see section 4).
- All folders on the path. Running any entry script from its own folder is
  enough, because each one adds the whole project via `genpath`.

There are two animation variants and two corresponding entry points:

**Option A — `runSkyGraphDashboard` (simplest, multi-panel dashboard).**
Self-contained: it trains any missing artifacts, configures the pipeline,
simulates Stage 13, validates the logs, and plays the multi-panel dashboard.

```matlab
cd <projectRoot>/scripts
runSkyGraphDashboard()                     % end-to-end, no arguments needed
```

**Option B — `runStage14Visualization` (full-flow 3-D playback).**
Simulates Stage 13 and plays the 3-D mission scene with a fixed camera.

```matlab
cd <projectRoot>/scripts
out = runStage14Visualization();
```

If you already have a valid simulation `out` in the workspace from either
script, you can replay the dashboard without re-simulating:

```matlab
runSkyGraphDashboard(out)                  % plays the dashboard on existing out
```

Playback controls (both variants): `Space` = pause/resume, `Esc` = stop.

---

## 6. Running the final evaluation (Stage 15)

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
