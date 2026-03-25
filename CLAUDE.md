# Robot Code Workspace

Multi-project workspace for Geodude robot manipulation research.
Setup: `git clone https://github.com/personalrobotics/robot-code && ./setup.sh`

## Projects

| Directory | Repo | Description |
|-----------|------|-------------|
| `geodude/` | personalrobotics/geodude | Main robot control, planning, execution |
| `geodude_assets/` | personalrobotics/geodude_assets | MuJoCo models for Geodude (UR5e + Robotiq) |
| `mj_manipulator/` | personalrobotics/mj_manipulator | Generic arm control: planning, execution, Cartesian control, grasping |
| `mj_manipulator_ros/` | personalrobotics/mj_manipulator_ros | Generic ROS 2 bridge: HardwareContext, mock node, trajectory conversion |
| `mj_environment/` | personalrobotics/mj_environment | MuJoCo environment wrapper |
| `prl_assets/` | personalrobotics/prl_assets | Reusable objects (cans, bins) |
| `pycbirrt/` | personalrobotics/pycbirrt | CBiRRT motion planner with TSR constraints |
| `tsr/` | personalrobotics/tsr | Task Space Regions for grasp/place planning |
| `asset_manager/` | personalrobotics/asset_manager | Loads objects from meta.yaml files |
| `mujoco_menagerie/` | google-deepmind/mujoco_menagerie | UR5e, Franka base models (cloned by setup.sh, not a Python package) |

## Conventions

- **Package manager**: Always use `uv` (never pip)
- **Layout**: `src/` layout with `src/{package_name}/`
- **Testing**: `uv run pytest tests/ -v`
- **Issues**: Use GitHub Issues for bugs, improvements, and tasks. Templates in `.github/ISSUE_TEMPLATE/`: `bug_report.md`, `improvement.md`, `task.md`.
- **CLAUDE.md updates**: Ask permission before changing this file.
