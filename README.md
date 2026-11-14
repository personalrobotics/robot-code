# robot-code

Multi-project workspace for MuJoCo-based robot manipulation research. Each robot's application sits on a small set of shared libraries, from robot packages and planning to robot-neutral grasping skills.

Built for the [Geodude](https://github.com/personalrobotics/geodude) bimanual robot (dual UR5e + Robotiq 2F-140), and for ADA (a Kinova JACO2 with its two-finger hand). The libraries underneath (ssrobot, ssgrasping, `sscbirrt`, `tsr`) are robot-neutral.

## Quick Start

```bash
git clone https://github.com/personalrobotics/robot-code
cd robot-code
./setup.sh        # clones all repos, syncs the shared workspace and geodude's and ada_mj's own environments
cd geodude
uv run geodude-recycling --seed 0 --mix 5             # headless
uv run geodude-recycling --seed 0 --mix 5 --viser     # browser viewer
```

Geodude 4.0 has its own uv environment, apart from the shared workspace; see the [geodude README](https://github.com/personalrobotics/geodude) for its robot API and recycling evaluation. The 3.x console, demos and LLM chat are at its `v3.1.0` tag.

ADA's application, ada_mj, also has its own environment: `cd ada_mj && uv run ada-lift --seed 0` grasps and lifts a cylinder (see the [ada_mj README](https://github.com/personalrobotics/ada_mj)). The earlier ada_mj, built on mj_manipulator, is at ada_mj's `v0.1.0` tag.

## Architecture

Each robot's application (geodude, ada_mj) is built on
[ssrobot](https://github.com/personalrobotics/ssrobot) and
[ssgrasping](https://github.com/personalrobotics/ssgrasping), pinned by each application's own lock:

| Layer | Owns |
| --- | --- |
| geodude, ada_mj | Each robot's task: which objects, the scene, the specs passed to each skill, how the run is driven, what counts as success |
| ssgrasping | Grasp, place, and drop skills written against robot-neutral contracts |
| ssrobot | Robot packages, the session and scene, planning contracts, the checked execution boundary, MuJoCo runtimes |
| sscbirrt, sstsr | Planning and timing over sets of starts, goals and constraints; grasp and placement templates |
| geodude_assets, ada_assets, prl_assets | Robot packages and object models |

mj_manipulator, the earlier arm-control library, and its ROS 2 bridge mj_manipulator_ros are retired
and archived ([mj_manipulator#182](https://github.com/personalrobotics/mj_manipulator/issues/182));
its README maps each capability to its successor.

## Packages

| Package | Description |
|---------|-------------|
| [geodude](https://github.com/personalrobotics/geodude) | Geodude 4.0: recycling, recovery, the robot API |
| [ada_mj](https://github.com/personalrobotics/ada_mj) | ADA's application: grasp and lift in open space |
| [ada_assets](https://github.com/personalrobotics/ada_assets) | MuJoCo models and the ssrobot package for ADA |
| [sscbirrt](https://github.com/personalrobotics/sscbirrt) | CBiRRT motion planner with TSR constraints |
| [sstsr](https://github.com/personalrobotics/sstsr) | Task Space Regions for grasp/place planning (imports as `tsr`) |
| [mj_environment](https://github.com/personalrobotics/mj_environment) | MuJoCo environment wrapper with object registry |
| [prl_assets](https://github.com/personalrobotics/prl_assets) | Reusable MuJoCo objects (cans, bins, spam cans, pop-tarts) |
| [asset_manager](https://github.com/personalrobotics/asset_manager) | Loads object geometry from `meta.yaml` files |
| [mj_viser](https://github.com/personalrobotics/mj_viser) | Web-based MuJoCo viewer built on Viser |
| [geodude_assets](https://github.com/personalrobotics/geodude_assets) | MuJoCo models for Geodude (UR5e + Robotiq 2F-140) |

## Conventions

- [MuJoCo group conventions](docs/mujoco-groups.md): which geom and site groups hold visuals, collision geometry, task frames, and engineering frames, in every asset model. Checked by `scripts/check_mujoco_groups.py`.

## Development

```bash
# Run tests for a specific package
cd geodude && uv run pytest tests/ -v

# The shared workspace's integration tests
uv run pytest tests/integration -v
```

### Real hardware (ROS 2)

**Planned** in ssrobot's hardware milestone: a ROS 2 runtime ([ssrobot#35](https://github.com/personalrobotics/ssrobot/issues/35)) and Geodude's hardware profiles ([ssrobot#36](https://github.com/personalrobotics/ssrobot/issues/36)).
