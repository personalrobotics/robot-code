# robot-code

Multi-project workspace for MuJoCo-based robot manipulation research. A curated set of focused libraries that compose into a full stack — from low-level joint control to high-level pick-and-place with behavior trees.

Built for the [Geodude](https://github.com/personalrobotics/geodude) bimanual robot (dual UR5e + Robotiq 2F-140), but the core libraries (`mj_manipulator`, `pycbirrt`, `tsr`) are robot-agnostic and work with any MuJoCo arm.

## Quick Start

```bash
git clone https://github.com/personalrobotics/robot-code
cd robot-code
./setup.sh        # clones all repos, runs uv sync
geodude --demo recycling
```

See the [geodude README](https://github.com/personalrobotics/geodude) for the full console guide, demos, and LLM chat.

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│  geodude                                                │
│  Bimanual robot: pickup/place primitives, behavior      │
│  trees, Vention linear bases, interactive console       │
├─────────────────────────────────────────────────────────┤
│  mj_manipulator                                         │
│  Generic arm control: planning, execution, Cartesian    │
│  control, grasping, collision checking                  │
├──────────────┬──────────────┬───────────────────────────┤
│  pycbirrt    │  tsr         │  mj_environment           │
│  CBiRRT      │  Task Space  │  MuJoCo environment       │
│  planner     │  Regions     │  wrapper + object registry│
├──────────────┴──────────────┼───────────────────────────┤
│  prl_assets + asset_manager │  geodude_assets            │
│  Reusable objects (cans,    │  Geodude MuJoCo models     │
│  bins) + YAML loader        │  (UR5e + Robotiq)          │
└─────────────────────────────┴───────────────────────────┘
```

### Key design decisions

- **ExecutionContext protocol** — same code runs in kinematic simulation, physics simulation, and on real hardware. `SimContext` for MuJoCo, `HardwareContext` (via ROS 2) for real robots.
- **Everything goes through the context** — no direct `mj_forward` or `data.qpos` writes during execution. Physics never freezes.
- **TSR-based grasping** — grasp and placement poses defined as Task Space Regions. The planner handles IK internally.
- **Behavior trees** — manipulation primitives composed with py_trees. Recovery is built into the tree structure.

## Packages

| Package | Description |
|---------|-------------|
| [geodude](https://github.com/personalrobotics/geodude) | Bimanual robot: high-level API, interactive console, demos, LLM chat |
| [mj_manipulator](https://github.com/personalrobotics/mj_manipulator) | Generic arm control: planning, trajectory execution, Cartesian control |
| [mj_manipulator_ros](https://github.com/personalrobotics/mj_manipulator_ros) | **WIP** — ROS 2 bridge: `HardwareContext`, mock node |
| [pycbirrt](https://github.com/personalrobotics/pycbirrt) | CBiRRT motion planner with TSR constraints |
| [tsr](https://github.com/personalrobotics/tsr) | Task Space Regions for grasp/place planning |
| [mj_environment](https://github.com/personalrobotics/mj_environment) | MuJoCo environment wrapper with object registry |
| [prl_assets](https://github.com/personalrobotics/prl_assets) | Reusable MuJoCo objects (cans, bins, potted meat) |
| [asset_manager](https://github.com/personalrobotics/asset_manager) | Loads object geometry from `meta.yaml` files |
| [geodude_assets](https://github.com/personalrobotics/geodude_assets) | MuJoCo models for Geodude (UR5e + Robotiq 2F-140) |

## Development

```bash
# Run tests for a specific package
cd geodude && uv run pytest tests/ -v

# Run all tests across the workspace
uv run pytest */tests/ -v
```

### Real hardware (ROS 2)

**Work in progress.** See [geodude#91](https://github.com/personalrobotics/geodude/issues/91) for the architecture and setup guide.
