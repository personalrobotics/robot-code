# robot-code

Multi-project workspace for MuJoCo-based robot manipulation research. A curated set of focused libraries that compose into a full stack — from low-level joint control to high-level pick-and-place with behavior trees.

Built for the [Geodude](https://github.com/personalrobotics/geodude) bimanual robot (dual UR5e + Robotiq 2F-140), but the core libraries (`mj_manipulator`, `pycbirrt`, `tsr`) are robot-agnostic and work with any MuJoCo arm.

## Quick Start

```bash
git clone https://github.com/personalrobotics/robot-code
cd robot-code
./setup.sh        # clones all repos, runs uv sync
```

Verify the stack end-to-end:

```bash
# Cartesian control demo (UR5e + Franka, no viewer)
uv run python mj_manipulator/demos/cartesian_control.py

# Recycling demo with viewer
uv run mjpython geodude/examples/recycle.py --physics

# Headless (CI-friendly)
uv run mjpython geodude/examples/recycle.py --headless --cycles 3
```

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│  geodude                                                │
│  Bimanual robot: pickup/place primitives, behavior      │
│  trees, Vention linear bases, scene setup               │
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
| [geodude](https://github.com/personalrobotics/geodude) | Bimanual robot: high-level API (`robot.pickup()`, `robot.place()`), Vention bases, behavior trees |
| [mj_manipulator](https://github.com/personalrobotics/mj_manipulator) | Generic arm control: planning, trajectory execution, Cartesian control, grasp-aware collision |
| [mj_manipulator_ros](https://github.com/personalrobotics/mj_manipulator_ros) | **WIP** — ROS 2 bridge: `HardwareContext`, MuJoCo mock node, trajectory conversion (requires system ROS 2) |
| [pycbirrt](https://github.com/personalrobotics/pycbirrt) | CBiRRT motion planner with TSR constraints |
| [tsr](https://github.com/personalrobotics/tsr) | Task Space Regions for grasp/place planning |
| [mj_environment](https://github.com/personalrobotics/mj_environment) | MuJoCo environment wrapper with object registry |
| [prl_assets](https://github.com/personalrobotics/prl_assets) | Reusable MuJoCo objects (cans, bins, potted meat) |
| [asset_manager](https://github.com/personalrobotics/asset_manager) | Loads object geometry from `meta.yaml` files |
| [geodude_assets](https://github.com/personalrobotics/geodude_assets) | MuJoCo models for Geodude (UR5e + Robotiq 2F-140) |

Robot base models (UR5e, Franka Panda) come from [mujoco_menagerie](https://github.com/google-deepmind/mujoco_menagerie), cloned by `setup.sh`.

## Workspace Layout

```
robot-code/
├── geodude/                  ← bimanual robot stack
├── mj_manipulator/           ← generic arm control
├── mj_manipulator_ros/       ← ROS 2 bridge (optional, needs system ROS 2)
├── mj_environment/           ← MuJoCo environment wrapper
├── pycbirrt/                 ← motion planner
├── tsr/                      ← Task Space Regions
├── prl_assets/               ← reusable objects
├── asset_manager/            ← YAML object loader
├── geodude_assets/           ← Geodude MuJoCo models
├── mujoco_menagerie/         ← cloned by setup.sh, not a Python package
├── pyproject.toml            ← uv workspace root
└── setup.sh                  ← bootstrap script
```

All packages are installed as editable in a shared `.venv`. Changes in any package are immediately visible to all others via `[tool.uv.sources]` workspace references.

## Using Individual Packages

Each package is independently installable outside the workspace:

```bash
uv add mj-manipulator @ git+https://github.com/personalrobotics/mj_manipulator
```

See each package's README for its standalone API.

## Interactive Console

IPython REPL with tab completion, introspection, and LLM-powered natural language control:

```bash
# Install chat dependencies
uv sync --extra chat

# Launch the console (kinematic mode)
uv run python geodude/examples/console.py --preset recycling

# With physics simulation and MuJoCo viewer
uv run mjpython geodude/examples/console.py --physics --viewer --preset recycling
```

```python
In [1]: robot.find_objects()
Out[1]: ['can_0', 'can_1', 'can_2', 'potted_meat_can_0']

In [2]: robot.pickup("can_0")       # pick up a specific object
In [3]: robot.place("recycle_bin")   # place in any bin
In [4]: robot.go_home()

In [5]: commands()                   # quick reference of all commands

# Natural language control (requires ANTHROPIC_API_KEY)
In [6]: chat('clear the table')
  → pickup({})
  ✓ Success
  → place({})
  ✓ Success
  ...
```

## Development

```bash
# Run tests for a specific package
cd geodude && uv run pytest tests/ -v

# Run a demo with the MuJoCo viewer (kinematic or physics)
uv run mjpython geodude/examples/recycle.py
uv run mjpython geodude/examples/recycle.py --physics

# Run all tests across the workspace
uv run pytest */tests/ -v
```

### Adding a new arm

`mj_manipulator` supports any MuJoCo arm. See [Adding a New Arm](https://github.com/personalrobotics/mj_manipulator#adding-a-new-arm) for the full guide. Pre-built: UR5e (6-DOF) and Franka Panda (7-DOF).

### Real hardware (ROS 2)

**Work in progress.** `mj_manipulator_ros` provides `HardwareContext` — the same `ExecutionContext` protocol backed by ROS 2 action servers. Scaffolded but not yet tested end-to-end. Requires system ROS 2 on Ubuntu. See [geodude#91](https://github.com/personalrobotics/geodude/issues/91) for the setup guide and architecture.
