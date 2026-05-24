<h1 align="center">📊 MATLAB Research Projects</h1>
<h3 align="center">Reinforcement Learning · Active Suspension · Multi-Agent Path Finding</h3>

<p align="center">
  <img src="https://img.shields.io/badge/MATLAB-R2025b-orange?logo=mathworks" />
  <img src="https://img.shields.io/badge/Simulink-Modelling-blue?logo=mathworks" />
  <img src="https://img.shields.io/badge/Reinforcement_Learning-TD3%2FDDPG-purple" />
  <img src="https://img.shields.io/badge/Control_Systems-Active_Suspension-green" />
  <img src="https://img.shields.io/badge/License-MIT-brightgreen" />
</p>

---

## 📌 Overview

This repository contains two major research projects developed in MATLAB/Simulink:

1. **🚗 Battery-Aware Active Suspension Control** — using Deep Reinforcement Learning (TD3/DDPG) to optimise vehicle ride comfort while managing battery degradation
2. **🤖 MAPF-RO MATLAB Visualizer** — 3D visualisation and simulation tool for the Multi-Agent Path Finding robot system

---

## 🚗 Project 1 — Battery-Aware Active Suspension with RL

### Overview
A deep reinforcement learning framework for **active vehicle suspension control** that simultaneously optimises:
- 🛋️ **Ride comfort** — minimising body acceleration
- 🔋 **Battery health** — reducing degradation from aggressive actuator use
- 🏎️ **Road handling** — maintaining tyre-road contact

### Key Files

| File | Description |
|---|---|
| `quarterCarModel.slx` | Quarter car Simulink model |
| `quarter_car_suspension.slx` | Full active suspension Simulink model |
| `quarterCarModel.m` | Quarter car MATLAB implementation |
| `quarter_car_matlab.m` | Main quarter car analysis script |
| `Environment.m` | RL training environment definition |
| `Train_TD3.m` | TD3 agent training script |
| `train_td3_final.m` | Final optimised TD3 training |
| `train_TD3_battery_v2.m` | Battery-aware TD3 v2 training |
| `step3_td3.m` | TD3 step training pipeline |
| `step3_battery_aware.m` | Battery-aware control step |
| `step3_battery_v2.m` | Battery-aware v2 pipeline |
| `Step1_passive.m` | Passive suspension baseline |
| `BatteryAware.m` | Battery degradation model |
| `battery_rl.m` | Battery RL integration |
| `reinforcement2.m` | Reinforcement learning utilities |
| `EvaluationBaseline.m` | Baseline evaluation script |
| `figures_generation.m` | Results figure generation |
| `figures_res.m` | Results visualisation |
| `MCE413.mlapp` | MATLAB App for suspension analysis |

### Results & Figures

<table>
<tr>
<td><img src="Fig1_Smooth_Body_Accel.png" width="200"/><br><em>Smooth Road Body Acceleration</em></td>
<td><img src="Fig2_Rough_Body_Accel.png" width="200"/><br><em>Rough Road Body Acceleration</em></td>
<td><img src="Fig3_Pothole_Body_Accel.png" width="200"/><br><em>Pothole Response</em></td>
</tr>
<tr>
<td><img src="Fig4_Battery_Accel_All_Roads.png" width="200"/><br><em>Battery Acceleration — All Roads</em></td>
<td><img src="Fig5_Degradation_All_Roads.png" width="200"/><br><em>Battery Degradation Comparison</em></td>
<td><img src="Fig8_Improvement_Percent.png" width="200"/><br><em>Improvement % over Passive</em></td>
</tr>
</table>

### How to Run

```matlab
% Step 1: Run passive baseline
run('Step1_passive.m')

% Step 2: Train TD3 agent
run('Train_TD3.m')

% Step 3: Evaluate battery-aware controller
run('step3_battery_aware.m')

% Step 4: Generate result figures
run('figures_generation.m')
```

### Algorithm — TD3 (Twin Delayed Deep Deterministic Policy Gradient)

```
┌─────────────────────────────────────────────────────┐
│              TD3 Active Suspension Loop              │
│                                                      │
│  Road Input → Quarter Car Model → State Observation  │
│       ↓                                              │
│  TD3 Agent (Actor-Critic Networks)                   │
│       ↓                                              │
│  Control Force → Actuator → Suspension               │
│       ↓                                              │
│  Reward = f(comfort, battery_health, handling)       │
└─────────────────────────────────────────────────────┘
```

**Reward Function** balances:
- Minimise body vertical acceleration (comfort)
- Minimise battery energy consumption (efficiency)
- Minimise suspension deflection (handling)

---

## 🤖 Project 2 — MAPF-RO MATLAB Visualizer

### Overview
A MATLAB-based **3D visualisation and simulation** tool for the [MAPF-RO](https://github.com/darish500/MAPF-RO) Multi-Agent Path Finding robot system. Provides an interactive environment to visualise and verify robot paths before deploying on ROS2.

### Key Files

| File | Description |
|---|---|
| `MAPFRO_3D_Visualizer.m` | 3D multi-robot path visualisation |
| `MAPFRO_App.m` | Interactive MATLAB App for MAPF-RO |
| `AStarPlanner.m` | A* path planning algorithm |
| `Robot.m` | Robot agent class definition |
| `Simulation.m` | Multi-robot simulation engine |
| `Environment.m` | Shared environment and obstacle map |
| `ObstacleManager.m` | Obstacle detection and avoidance |

### How to Run

```matlab
% Launch the MAPF-RO 3D Visualizer
run('MAPFRO_3D_Visualizer.m')

% Launch the interactive App
run('MAPFRO_App.m')

% Run a multi-robot simulation
run('Simulation.m')
```

### Related Repository
🔗 [MAPF-RO — ROS2 Implementation](https://github.com/darish500/MAPF-RO)

---

## 🛠️ Requirements

- **MATLAB R2022a or later** (R2025b recommended)
- **Simulink** — for quarter car models
- **Reinforcement Learning Toolbox** — for TD3/DDPG training
- **Control System Toolbox** — for suspension analysis
- **Deep Learning Toolbox** — for neural network agents

Install required toolboxes via MATLAB Add-On Explorer.

---

## 📁 Repository Structure

```
MATLAB-/
├── 🚗 Suspension & RL
│   ├── quarterCarModel.slx        # Quarter car Simulink model
│   ├── quarter_car_suspension.slx # Active suspension model
│   ├── Train_TD3.m                # TD3 training
│   ├── train_td3_final.m          # Final trained pipeline
│   ├── BatteryAware.m             # Battery degradation model
│   ├── Environment.m              # RL environment
│   ├── EvaluationBaseline.m       # Baseline comparison
│   └── figures_generation.m       # Results figures
│
├── 🤖 MAPF-RO Visualizer
│   ├── MAPFRO_3D_Visualizer.m     # 3D path visualiser
│   ├── MAPFRO_App.m               # Interactive app
│   ├── AStarPlanner.m             # A* algorithm
│   ├── Robot.m                    # Robot class
│   ├── Simulation.m               # Simulation engine
│   ├── Environment.m              # Environment map
│   └── ObstacleManager.m          # Obstacle handling
│
├── 📊 Results & Figures
│   ├── Fig1_Smooth_Body_Accel.png
│   ├── Fig2_Rough_Body_Accel.png
│   ├── Fig5_Degradation_All_Roads.png
│   └── ...
│
└── 🎓 MCE413 Course App
    └── MCE413.mlapp               # MATLAB App Designer project
```

---

## 🌍 Real-World Applications

- 🚗 **Automotive** — Smart active suspension for electric vehicles
- 🔋 **EV Battery Management** — RL-optimised energy-aware control
- 🤖 **Robotics** — Multi-agent coordination and path planning
- 🏭 **Industrial Automation** — Vibration control in machinery
- 🛸 **Aerospace** — Vibration isolation for sensitive payloads

---

## 📄 License

MIT License — **darish500** · [github.com/darish500](https://github.com/darish500)

---

## 👤 Author

**Darish** — AI Engineer, Robotics & Control Systems Developer 🇳🇬  
📧 rasakkhalid145@gmail.com  
🔗 [github.com/darish500](https://github.com/darish500)
