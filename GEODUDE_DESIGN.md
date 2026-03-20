# Geodude: High-Level API for Bimanual Manipulation

## Overview

This document captures the design for a high-level Python API to program the Geodude bimanual robot for object manipulation tasks. The goal is to make it as easy as HERBPy was for OpenRAVE, but built on MuJoCo and modern Python tooling.

## Existing Repositories

The following repositories already exist in `/Users/siddh/code/robot-code/` and should be leveraged:

| Repository | Purpose | Status |
|------------|---------|--------|
| `geodude_assets` | MuJoCo models for bimanual UR5e + grippers (Robotiq 2F-140, Psyonic Ability Hand) | Ready |
| `asset_manager` | Unified object registry (metadata, perception aliases, simulator configs) | Ready |
| `mj_environment` | Dynamic MuJoCo scene management (object lifecycle without model reload) | Ready |
| `tsr` | Task Space Regions for manipulation constraints (grasps, placements) | Ready |
| `pycbirrt` | Constrained Bi-directional RRT planner with TSR support | Ready |
| `pyplanning` | Lightweight planning framework (interfaces, snap planner) | Minimal |
| `pyrave` | Unified robotics runtime (future orchestrator) | Early stage |

## Design Principles

1. **Make common cases trivial** - `robot.right_arm.pick('cup')` should just work
2. **Don't hide components** - Users can access TSRs, planners, MuJoCo model when needed
3. **Separation of concerns** - Planner doesn't know about grasping; collision checker handles it
4. **Physics-based grasping** - MuJoCo contact/friction holds objects, no kinematic attachment
5. **Layer 1 first** - Get happy path working before adding robustness/verification

## Target API

### Initialization

```python
from geodude import Geodude

robot = Geodude()  # Loads MuJoCo model, initializes environment
# or
robot = Geodude.from_config('config/default.yaml')
```

### Simple Pick and Place

```python
# High-level (common case)
robot.right_arm.pick('cup')
robot.right_arm.place('cup', on='table')

# Bimanual
robot.bimanual.pick('large_box')
```

### Named Configurations

```python
robot.left_arm.go_to('home')
robot.right_arm.go_to('ready')
robot.go_to('home')  # Both arms
```

### More Control When Needed

```python
# Get TSRs explicitly
grasp_tsrs = robot.get_grasp_tsrs('cup', variants=['side', 'top'])

# Plan without executing
path = robot.right_arm.plan_to_tsrs(grasp_tsrs)

# Execute
robot.right_arm.execute(path)

# Gripper control
robot.right_arm.close_gripper()
robot.right_arm.open_gripper()
```

### Vention Base Control

```python
# Access linear actuators (separate from arm control)
robot.left_base.height      # Current height in meters (0-0.5)
robot.right_base.height

# Set height directly (no collision check)
robot.left_base.set_height(0.3)

# Move with collision checking (checks arm at discrete steps)
success = robot.left_base.move_to(0.4, check_collisions=True)
```

### Access Underlying Components

```python
robot.model       # MuJoCo MjModel
robot.data        # MuJoCo MjData
robot.env         # mj_environment.Environment
robot.assets      # asset_manager.AssetManager
robot.left_base   # VentionBase (left linear actuator)
robot.right_base  # VentionBase (right linear actuator)
```

## Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                              Geodude                                     │
│  - Central robot interface                                               │
│  - Manages arms, bases, environment, grasp state                         │
│  - Provides high-level pick/place API                                    │
└─────────────────────────────────────────────────────────────────────────┘
         │              │                │                    │
         ▼              ▼                ▼                    ▼
┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐
│     Arm     │  │ VentionBase │  │GraspManager │  │    Environment      │
│ -left/right │  │ -left/right │  │ -tracks what│  │  - mj_environment   │
│ -FK/IK      │  │ -height     │  │  is grasped │  │  - object spawn     │
│ -plan/exec  │  │ -move_to()  │  │ -updates    │  │  - asset_manager    │
│ -gripper    │  │ -collision  │  │  collision  │  │                     │
└─────────────┘  └─────────────┘  └─────────────┘  └─────────────────────┘
         │                    │
         ▼                    ▼
┌─────────────────────────────────────────────────────────────┐
│                  GraspAwareCollisionChecker                 │
│  - Checks arm-environment collisions only                  │
│  - Self-collision handled by MuJoCo <exclude> tags         │
│  - Queries GraspManager for expected grasp contacts        │
└─────────────────────────────────────────────────────────────┘
         │
         ▼
┌─────────────────────────────────────────────────────────────┐
│                        pycbirrt                             │
│  - CBiRRT planner                                          │
│  - Takes: robot_model, ik_solver, collision_checker        │
│  - Plans paths using TSRs                                  │
│  - Doesn't know about grasping                             │
└─────────────────────────────────────────────────────────────┘
         │
         ▼
┌─────────────────────────────────────────────────────────────┐
│                          tsr                                │
│  - Task Space Region definitions                           │
│  - Grasp/place templates per object type                   │
│  - Sampling, distance computation                          │
└─────────────────────────────────────────────────────────────┘
```

## Collision Architecture

### Self-Collision: Handled by MuJoCo Model

The MuJoCo model (`geodude.xml`) defines `<exclude>` contact pairs for:
- Adjacent arm links (shoulder↔upper_arm, etc.)
- Gripper internal contacts (fingers, pads, drivers)

This means **geodude's Python code does not need to track self-collision pairs**. The collision checkers simply trust MuJoCo's native filtering.

### Grasped Object Collision Filtering

When the gripper grasps an object:
1. **Gripper pads ↔ Object**: Must allow contact (for stable grasp)
2. **Object ↔ Environment**: Must allow contact (table, other objects)
3. **Object ↔ Rest of robot arm**: Must PREVENT contact (false collisions during planning)

### Solution: Collision Groups via contype/conaffinity

MuJoCo uses bitmasks for collision filtering. We define:

| Entity | contype | conaffinity | Collides with |
|--------|---------|-------------|---------------|
| Robot arm | 1 | 1 | Arms, objects (normal) |
| Gripper pads | 3 | 3 | Arms, objects (normal), objects (grasped) |
| Object (normal) | 1 | 1 | Everything |
| Object (grasped) | 2 | 2 | Gripper pads, environment, NOT arm |

### Implementation

```python
class GraspManager:
    def __init__(self, model, data):
        self.model = model
        self.data = data
        self.grasped = {}  # object_name -> arm_name

    def mark_grasped(self, object_name: str, arm: str):
        """Called when gripper closes on object."""
        self.grasped[object_name] = arm
        self._set_object_collision_group(object_name, grasped=True)

    def mark_released(self, object_name: str):
        """Called when gripper releases object."""
        self.grasped.pop(object_name, None)
        self._set_object_collision_group(object_name, grasped=False)

    def _set_object_collision_group(self, object_name: str, grasped: bool):
        body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, object_name)
        for geom_id in range(self.model.ngeom):
            if self.model.geom_bodyid[geom_id] == body_id:
                if grasped:
                    self.model.geom_contype[geom_id] = 2
                    self.model.geom_conaffinity[geom_id] = 2
                else:
                    self.model.geom_contype[geom_id] = 1
                    self.model.geom_conaffinity[geom_id] = 1
```

### Integration with Planner

pycbirrt doesn't change. It receives a `CollisionChecker` that checks arm-environment collisions:

```python
class GraspAwareCollisionChecker:
    def __init__(self, model, data, joint_names, grasp_manager):
        self.model = model
        self.data = data
        self.grasp_manager = grasp_manager
        self._arm_body_ids = self._build_arm_body_ids(joint_names)

    def is_valid(self, q: np.ndarray) -> bool:
        # Set configuration and run forward kinematics
        # ...
        return self._count_invalid_contacts() == 0

    def _count_invalid_contacts(self) -> int:
        # Self-collision: handled by MuJoCo <exclude> tags
        # Only check arm-environment contacts
        for contact in self.data.contact:
            body1_is_arm = body1 in self._arm_body_ids
            body2_is_arm = body2 in self._arm_body_ids

            # Skip same-arm contacts (MuJoCo handles via <exclude>)
            if body1_is_arm and body2_is_arm:
                continue

            # Skip non-arm contacts (e.g., object on table)
            if not body1_is_arm and not body2_is_arm:
                continue

            # Arm-environment contact - check if expected grasp
            if self._is_expected_grasp_contact(body1, body2):
                continue

            return 1  # Invalid collision
        return 0
```

## Gripper and Grasp State Flow

In MuJoCo, grasping is physics-based (friction holds the object). We don't need explicit kinematic attachment.

### close_gripper() behavior:

```python
def close_gripper(self):
    # 1. Command gripper actuator to close
    self._set_gripper_ctrl(closed=True)

    # 2. Step simulation until gripper settles
    self._step_until_settled()

    # 3. Detect what (if anything) is grasped via contacts
    grasped_obj = self._detect_grasped_object()

    # 4. Update collision groups if object grasped
    if grasped_obj:
        self.grasp_manager.mark_grasped(grasped_obj, self.arm_name)
```

### open_gripper() behavior:

```python
def open_gripper(self):
    # 1. Command gripper actuator to open
    self._set_gripper_ctrl(closed=False)

    # 2. Step simulation until gripper settles
    self._step_until_settled()

    # 3. Mark any previously grasped object as released
    for obj in self.grasp_manager.get_objects_held_by(self.arm_name):
        self.grasp_manager.mark_released(obj)
```

### No separate grab()/release() needed

The grasp state is inferred from physics. When gripper closes on an object, we detect it and update collision groups automatically.

## Trajectory Execution Architecture

### Goal: Sim-to-Real with a Boolean Switch

The north star is making it trivial to switch between simulation and real robot execution. The same trajectory should execute identically in both environments.

### Architecture Overview

```
Path (geometric)          Trajectory (time-parameterized)       Execution
  [q0, q1, q2, ...]  -->  Trajectory(positions, velocities,  -->  Executor.execute(traj)
                           accelerations, timestamps)
                                    ^
                                    |
                              TOPP-RA retiming
                          (respects vel/acc limits)
```

### Trajectory Class

Time-parameterized trajectory with positions, velocities, accelerations at discrete timestamps:

```python
@dataclass
class Trajectory:
    """Time-parameterized robot trajectory."""
    timestamps: np.ndarray      # (N,) seconds from start
    positions: np.ndarray       # (N, dof) joint positions in radians
    velocities: np.ndarray      # (N, dof) joint velocities in rad/s
    accelerations: np.ndarray   # (N, dof) joint accelerations in rad/s²

    @classmethod
    def from_path(
        cls,
        path: list[np.ndarray],
        vel_limits: np.ndarray,
        acc_limits: np.ndarray,
    ) -> "Trajectory":
        """Create time-optimal trajectory from geometric path using TOPP-RA."""
```

### Executor Abstraction

```python
class Executor(Protocol):
    """Protocol for trajectory execution."""
    def execute(self, trajectory: Trajectory) -> bool: ...

class KinematicExecutor:
    """Execute trajectories via direct position setting (no physics).
    Perfect tracking, useful for visualization and planning validation.
    Sets qpos directly at 125 Hz, no physics simulation.
    """

class PhysicsExecutor:
    """Execute trajectories in MuJoCo physics simulation.
    Uses position control with velocity feedforward at 125 Hz.
    Reports tracking error, detects collisions during execution.
    """

class RealExecutor:
    """Execute trajectories on real robot via ros_control/ur_rtde.
    Streams trajectory points to the robot controller.
    """
```

### Kinematic vs Physics Execution

Two execution modes serve different purposes:

| Mode | Physics | Tracking | Use Case |
|------|---------|----------|----------|
| `kinematic` | No | Perfect (0°) | Visualization, planning validation |
| `physics` | Yes | ~1-3° | Realistic simulation, actuator tuning |

The `arm_planning.py` demo supports both via `--executor` flag.

### UR5e Kinematic Limits

Velocity and acceleration limits are defined in [config.py](geodude/src/geodude/config.py:15) as part of the configuration system:

```python
@dataclass
class KinematicLimits:
    """Velocity and acceleration limits for trajectory planning."""
    velocity: np.ndarray  # rad/s per joint
    acceleration: np.ndarray  # rad/s² per joint

    @classmethod
    def ur5e_default(cls, vel_scale: float = 0.5, acc_scale: float = 0.5):
        """UR5e limits from datasheet: [3.14, 3.14, 3.14, 6.28, 6.28, 6.28] rad/s."""
        # Applied with 50% safety scaling by default for conservative lab operation
```

**Why not in MuJoCo XML?** MuJoCo doesn't support velocity/acceleration limit attributes in the joint definition. Position limits (`range`) are in the XML, but vel/acc limits are planning constraints from the manufacturer's specifications, not physics parameters.

### Key Design Decisions

1. **Control frequency**: 125 Hz (8ms) to match UR5e internal servo rate
2. **TOPP-RA retiming**: Time-optimal path parameterization respecting kinematic limits
3. **Dense trajectory**: Waypoints at control frequency (simpler than spline evaluation in loop)
4. **Same API**: `executor.execute(trajectory)` works for both sim and real

## Layer 1 vs Layer 2

### Layer 1 (Initial Implementation)
- Assume grasps succeed if planned correctly
- Assume releases work when gripper opens
- `pick()` / `place()` return success based on planning success
- No grasp verification or recovery

### Layer 2 (Future Enhancement)
- Verify grasp via contact/force sensing after close
- Detect slip during trajectory execution
- Recovery behaviors (re-grasp, retry different TSR)
- `pick(..., verify=True)` option

## Package Structure

```
geodude/
├── src/geodude/
│   ├── __init__.py           # Lazy imports, public API
│   ├── robot.py              # Geodude main class
│   ├── arm.py                # Arm class with IK, planning, pick/place
│   ├── vention_base.py       # VentionBase class for linear actuators
│   ├── gripper.py            # Gripper control + grasp detection
│   ├── grasp_manager.py      # Tracks grasp state, updates collision groups
│   ├── collision.py          # GraspAwareCollisionChecker, SimpleCollisionChecker
│   ├── config.py             # All config dataclasses (YAML support, kinematic limits)
│   ├── tsr_utils.py          # TSR creation utilities for manipulation
│   ├── trajectory.py         # Trajectory class + TOPP-RA retiming
│   └── executor.py           # SimExecutor, RealExecutor
├── tests/                    # 136 tests across 8 files
├── examples/
│   ├── arm_planning.py           # Full demo: forked RRT, TOPP-RA, dual-arm + bases
│   ├── basic_movement.py         # Visual demo with viewer
│   ├── symmetric_grasp_test.py   # IK testing at different heights
│   ├── named_config_planning.py  # CBiRRT planning demo
│   └── interactive_viewer.py     # Basic visualization
├── snapshots/                # Test snapshots
└── pyproject.toml
```

## Changes to Existing Repos

### geodude_assets
- Add collision group defaults to XML:
  - Robot arm geoms: `contype="1" conaffinity="1"`
  - Gripper pads: `contype="3" conaffinity="3"`
- Document collision group semantics
- ✅ **Actuator gains tuned** (January 2026):
  - `ur5e.xml`: Increased gains 4x (2000→8000 for large joints, 500→2000 for wrist)
  - Force limits doubled (150→300 N, 28→56 N) to accommodate higher gains
  - Damping doubled (400→800, 100→200) for stability
  - Result: Better trajectory tracking in physics simulation (~1-3° max error)
- ✅ **Default camera added** to `geodude.xml`:
  - `<camera name="main" pos="0.085 -2.31 0.25" xyaxes="1 0 0 0 0.446 0.895"/>`
  - Side view from left, slightly elevated for demo visualization

### mj_environment
- Add collision group field to object meta.yaml
- Default objects to `contype="1" conaffinity="1"`
- Add method to update object collision groups at runtime

### tsr
- Add geodude-specific TSR templates for common objects
- Ensure templates work with asset_manager object types

### pycbirrt
- No changes needed (clean separation of concerns)

## TSR Library Integration

TSRs for grasping/placing are looked up by object type:

```python
# In Geodude class
def get_grasp_tsrs(self, object_name: str, variants: list[str] = None) -> list[TSR]:
    obj_type = self.env.get_object_type(object_name)  # e.g., 'cup'
    obj_pose = self.env.get_object_pose(object_name)

    templates = load_grasp_templates(obj_type, variants)
    return [t.instantiate(obj_pose) for t in templates]
```

Templates stored in `tsr/templates/grasps/` are keyed by object type matching asset_manager categories.

## Example Usage

```python
from geodude import Geodude

# Initialize
robot = Geodude()

# Add objects to scene
robot.env.add_object('cup', type='cup', position=[0.5, 0, 0.8])
robot.env.add_object('plate', type='plate', position=[0.3, 0.2, 0.8])

# Simple pick and place
robot.right_arm.go_to('home')
robot.right_arm.pick('cup')
robot.right_arm.place('cup', on='plate')

# Or with more control
grasp_tsrs = robot.get_grasp_tsrs('cup', variants=['side'])
path = robot.right_arm.plan_to_tsrs(grasp_tsrs)
robot.right_arm.execute(path)
robot.right_arm.close_gripper()  # Auto-detects grasp, updates collision groups

place_tsrs = robot.get_place_tsrs('cup', on='plate')
path = robot.right_arm.plan_to_tsrs(place_tsrs)  # Planner uses grasp-aware collision
robot.right_arm.execute(path)
robot.right_arm.open_gripper()  # Auto-releases, restores collision groups

# Bimanual (future)
robot.bimanual.pick('large_box')
```

## pycbirrt UR5e Example Integration

The `pycbirrt/examples/ur5e_mujoco.py` example demonstrates a complete motion planning pipeline for a UR5e arm with Robotiq gripper. Geodude reuses and adapts these patterns for its bimanual setup.

### What We Reuse from the Example

#### 1. EAIK Analytical IK Solver

The example shows how to use EAIK for fast, multi-solution IK:

```python
from pycbirrt.backends.eaik import EAIKSolver

# Pre-configured for UR5e DH parameters
ik_solver = EAIKSolver.for_ur5e(
    joint_limits=robot.joint_limits,
    collision_checker=collision_checker,
)

# Returns up to 8 analytical solutions
solutions = ik_solver.solve_valid(target_pose)
```

**Why this matters for geodude:**
- Analytical IK is ~100x faster than differential IK
- Multiple solutions let the planner explore different arm configurations
- Critical for TSR planning where we need to try many goal poses

#### 2. MuJoCo Backend Components

The example uses three key backend classes that geodude adapts:

```python
from pycbirrt.backends.mujoco import (
    MuJoCoRobotModel,        # FK via site pose
    MuJoCoCollisionChecker,  # Contact-based collision checking
    MuJoCoIKSolver,          # Differential IK (fallback)
)
```

**Geodude equivalents:**
- `Arm` class already provides FK via `get_ee_pose()`
- `GraspAwareCollisionChecker` extends `MuJoCoCollisionChecker` with grasp state
- Will wrap `EAIKSolver` for IK

#### 3. CBiRRT Planner Integration

The example shows the planner setup pattern:

```python
from pycbirrt import CBiRRT, CBiRRTConfig

config = CBiRRTConfig(
    max_iterations=5000,
    step_size=0.2,
    goal_bias=0.1,
)

planner = CBiRRT(
    robot=robot_model,
    ik_solver=ik_solver,
    collision_checker=collision_checker,
    config=config,
)

# Plan to TSR goals
path = planner.plan(start_config, goal_tsrs=[grasp_tsr], seed=42)
```

#### 4. TSR Template Pattern

The example defines TSR templates for grasping and placement:

```python
from tsr.core.tsr_template import TSRTemplate

def create_gripper_grasp_cylinder_template() -> TSRTemplate:
    """Top-down grasp with gripper z-axis pointing down."""
    standoff = 0.25  # Height above object for attachment_site

    # Gripper frame: z points DOWN (180° rotation around x)
    Tw_e = np.array([
        [1, 0, 0, 0],
        [0, -1, 0, 0],
        [0, 0, -1, standoff],
        [0, 0, 0, 1],
    ])

    # Allow yaw rotation, small xy tolerance
    Bw = np.array([
        [-0.02, 0.02],      # x tolerance
        [-0.02, 0.02],      # y tolerance
        [0, 0.05],          # z: can be higher
        [-0.01, 0.01],      # roll tolerance
        [-0.01, 0.01],      # pitch tolerance
        [-np.pi, np.pi],    # yaw: any rotation
    ])

    return TSRTemplate(T_ref_tsr=np.eye(4), Tw_e=Tw_e, Bw=Bw, ...)

# Instantiate at object pose
grasp_tsr = grasp_template.instantiate(T_object_world)
```

### Integration Plan for Geodude

#### Phase 1: Add IK Solver to Arm

```python
# In arm.py
class Arm:
    def _get_ik_solver(self) -> EAIKSolver:
        """Lazy-load EAIK solver for this arm."""
        if self._ik_solver is None:
            from pycbirrt.backends.eaik import EAIKSolver
            self._ik_solver = EAIKSolver.for_ur5e(
                joint_limits=self.get_joint_limits(),
                collision_checker=self._get_collision_checker(),
            )
        return self._ik_solver

    def solve_ik(self, pose: np.ndarray) -> list[np.ndarray]:
        """Solve IK for end-effector pose.

        Returns:
            List of valid joint configurations (up to 8 for UR5e)
        """
        return self._get_ik_solver().solve_valid(pose)
```

#### Phase 2: Add Robot Model Adapter

pycbirrt expects a `RobotModel` protocol. Create an adapter:

```python
# In arm.py or new file planning.py
class ArmRobotModel:
    """Adapter to make Arm compatible with pycbirrt RobotModel protocol."""

    def __init__(self, arm: Arm):
        self.arm = arm

    @property
    def dof(self) -> int:
        return self.arm.dof

    @property
    def joint_limits(self) -> tuple[np.ndarray, np.ndarray]:
        return self.arm.get_joint_limits()

    def forward_kinematics(self, q: np.ndarray) -> np.ndarray:
        self.arm.set_joint_positions(q)
        return self.arm.get_ee_pose()
```

#### Phase 3: Integrate CBiRRT Planner

```python
# In arm.py
class Arm:
    def _get_planner(self) -> CBiRRT:
        """Lazy-load CBiRRT planner."""
        if self._planner is None:
            from pycbirrt import CBiRRT, CBiRRTConfig

            config = CBiRRTConfig(
                max_iterations=5000,
                step_size=0.2,
                goal_bias=0.1,
            )

            self._planner = CBiRRT(
                robot=ArmRobotModel(self),
                ik_solver=self._get_ik_solver(),
                collision_checker=self._get_collision_checker(),
                config=config,
            )
        return self._planner

    def plan_to_configuration(self, q_goal: np.ndarray) -> list[np.ndarray] | None:
        """Plan path to goal configuration using CBiRRT.

        IMPORTANT: Pass goal config directly to planner's `goal` parameter,
        NOT as a point TSR via `goal_tsrs`. Converting config→FK→TSR→IK is
        redundant and can produce different IK solutions than the original.
        """
        q_start = self.get_joint_positions()
        planner = self._get_planner()
        path = planner.plan(start=q_start, goal=q_goal)  # Direct config, no TSR

        # Restore state - planner corrupts it during collision checking
        self.set_joint_positions(q_start)
        return path

    def plan_to_tsrs(self, tsrs: list, timeout: float = 30.0) -> list[np.ndarray] | None:
        """Plan path to reach any of the given TSRs."""
        q_start = self.get_joint_positions()
        planner = self._get_planner()
        path = planner.plan(
            q_start,
            goal_tsrs=tsrs,
            timeout=timeout,
        )

        # Restore state - planner corrupts it during collision checking
        self.set_joint_positions(q_start)
        return path
```

#### Phase 4: TSR Template Library

Create geodude-specific TSR templates:

```python
# In geodude/tsr_templates.py
from tsr.core.tsr_template import TSRTemplate

def create_robotiq_2f140_grasp_template(variant: str = "top") -> TSRTemplate:
    """Grasp template for Robotiq 2F-140 gripper on geodude.

    Args:
        variant: 'top' for top-down, 'side' for lateral grasp
    """
    if variant == "top":
        # Top-down grasp (z pointing down)
        standoff = 0.20  # Adjusted for 2F-140 geometry
        Tw_e = np.array([
            [1, 0, 0, 0],
            [0, -1, 0, 0],
            [0, 0, -1, standoff],
            [0, 0, 0, 1],
        ])
    elif variant == "side":
        # Side grasp (approach from side)
        # ... different transform

    return TSRTemplate(...)

# In Geodude class
def get_grasp_tsrs(self, object_name: str, variants: list[str] = None) -> list[TSR]:
    """Get grasp TSRs for an object."""
    obj_pose = self.get_object_pose(object_name)
    variants = variants or ["top", "side"]

    tsrs = []
    for variant in variants:
        template = create_robotiq_2f140_grasp_template(variant)
        tsrs.append(template.instantiate(obj_pose))
    return tsrs
```

### Key Differences from UR5e Example

| Aspect | UR5e Example | Geodude |
|--------|--------------|---------|
| Arms | Single arm | Bimanual (left + right) |
| Gripper | Robotiq 2F-85 | Robotiq 2F-140 |
| Collision | Simple checker | GraspAwareCollisionChecker |
| Scene | Static cylinder | Dynamic via mj_environment |
| EE Site | `attachment_site` | `{arm}_ur5e/gripper_attachment_site` |
| Joint prefix | None | `left_ur5e/`, `right_ur5e/` |

### Visualization Reuse

The example's visualization code can be adapted for geodude:

```python
# Render path to video
render_to_video(model, data, path, joint_names, "output.mp4")

# Interactive viewer
visualize_interactive(model, data, path, joint_names)
```

## Configuration

Configuration is organized by concern, with clear provenance for planner-specific parameters.

```python
@dataclass
class ArmConfig:
    """Configuration for a single UR5e arm."""
    name: str                    # "left" or "right"
    joint_names: list[str]       # 6 UR5e joint names
    ee_site: str                 # MuJoCo site for end-effector
    gripper_actuator: str        # Gripper actuator name
    gripper_bodies: list[str]    # Bodies for collision filtering

@dataclass
class VentionBaseConfig:
    """Configuration for a Vention linear actuator."""
    name: str                    # "left" or "right"
    joint_name: str              # "left_arm_linear_vention"
    actuator_name: str           # "left_linear_actuator"
    height_range: tuple[float, float] = (0.0, 0.5)  # meters
    collision_check_resolution: float = 0.01  # meters between checks

@dataclass
class CBiRRTConfig:
    """CBiRRT-specific planner parameters.

    These map directly to pycbirrt.CBiRRTConfig.
    """
    max_iterations: int = 5000
    goal_bias: float = 0.1
    tsr_samples: int = 50        # Max pose samples to try from TSR
    num_tree_roots: int = 5      # Target number of root configs per tree

@dataclass
class PlanningConfig:
    """General planning parameters."""
    step_size: float = 0.2       # radians - path discretization
    timeout: float = 30.0        # seconds
    cbirrt: CBiRRTConfig = field(default_factory=CBiRRTConfig)

@dataclass
class ExecutionConfig:
    """Trajectory execution parameters."""
    interpolation_steps: int = 50    # steps between waypoints
    gripper_settle_steps: int = 100  # simulation steps for gripper open/close

@dataclass
class GeodudConfig:
    """Full robot configuration."""
    model_path: Path
    left_arm: ArmConfig
    right_arm: ArmConfig
    left_base: VentionBaseConfig
    right_base: VentionBaseConfig
    planning: PlanningConfig = field(default_factory=PlanningConfig)
    execution: ExecutionConfig = field(default_factory=ExecutionConfig)
    named_poses: dict[str, dict[str, list[float]]] = field(default_factory=dict)
```

## Implementation Status

### Completed

- [x] Core classes: `Geodude`, `Arm`, `Gripper`, `GraspManager`, `GraspAwareCollisionChecker`
- [x] Configuration system with YAML support (`GeodudConfig`, `ArmConfig`)
- [x] Collision group filtering for grasped objects (contype/conaffinity)
- [x] Grasp detection via gripper contacts
- [x] EAIK analytical IK solver integration
  - `Arm.inverse_kinematics(pose)` returns up to 8 solutions
  - `Arm.forward_kinematics_eaik(q)` for fast FK
  - Coordinate transforms between MuJoCo EE site and EAIK DH frames
  - `ArmRobotModel` and `ArmIKSolver` adapters for pycbirrt
- [x] CBiRRT motion planning integration
  - `Arm.plan_to_configuration(q_goal)` for point-to-point planning
  - `Arm.plan_to_tsrs(tsrs)` for TSR-based planning
  - `GraspAwareCollisionChecker` passed to planner
- [x] TSR utilities (`geodude/tsr_utils.py`)
  - `create_top_grasp_tsr()`, `create_side_grasp_tsr()`
  - `create_place_tsr()`, `create_lift_tsr()`, `create_retract_tsr()`
  - `create_approach_tsr()` for pre-grasp/pre-place poses
- [x] Full `pick()` and `place()` implementations
  - Approach → grasp → lift sequence
  - Approach → place → release → retract sequence
- [x] VentionBase class for linear actuator control
  - `robot.left_base`, `robot.right_base` properties
  - `height` property, `set_height()`, `move_to(check_collisions=True)`
  - Collision checking along interpolated path
- [x] Simplified collision checking architecture
  - Self-collision filtering delegated to MuJoCo `<exclude>` tags in model
  - `GraspAwareCollisionChecker` only checks arm-environment collisions
  - Removed redundant adjacency/gripper body tracking from Python
- [x] Trajectory retiming and execution layer
  - `Trajectory` class with TOPP-RA time-optimal path parameterization
  - `SimExecutor` for MuJoCo simulation at 125 Hz (matches UR5e servo rate)
  - `KinematicLimits` in config with UR5e velocity/acceleration limits
  - Arm.execute() automatically converts paths to time-optimal trajectories
- [x] Test suite (148 tests) using real geodude_assets model
- [x] Examples directory with 5 runnable scripts
  - `arm_planning.py` - Full demo with forked RRT, TOPP-RA retiming, dual-arm + base movement
  - `basic_movement.py` - Visual demo with random movements
  - `symmetric_grasp_test.py` - IK solving for symmetric grasping
  - `named_config_planning.py` - Planning between named configurations
  - `interactive_viewer.py` - Basic visualization
- [x] Git repository initialized
- [x] Planning API bug fixes (January 2026)
  - `plan_to_configuration()`: Pass goal config directly to planner instead of
    converting to point TSR and re-solving IK (which could produce different solutions)
  - `plan_to_tsrs()`: Add state restoration after planning (planner corrupts state
    during collision checking)
  - `pick()`/`place()`: Use `_get_ee_pose_at_config()` instead of temporarily
    teleporting robot to compute FK (avoids visual artifacts)
  - `pick()`/`place()`: Return failure on re-planning failure instead of executing
    stale path from wrong start position
- [x] Actuator gains tuning (January 2026)
  - Increased UR5e actuator gains 4x in `ur5e.xml` for better trajectory tracking
  - Doubled force limits to accommodate higher gains (150→300 N for large joints)
  - Doubled damping for stability (400→800 for large joints)
  - Testing showed ~18% improvement in tracking error with 2x gains; 4x selected for margin
- [x] Dual execution modes (January 2026)
  - `KinematicExecutor`: Direct qpos setting, perfect tracking, no physics
  - `PhysicsExecutor`: Full MuJoCo physics with position+velocity control
  - Different APIs: `set_position(q)` vs `set_target(q, qd)` + `step()`
- [x] Forked parallel planning (January 2026)
  - `plan_with_forks()`: Run multiple RRT instances with different seeds
  - First successful plan wins (process-based parallelism)
  - Configurable via `--forks N` in arm_planning demo
- [x] Default camera view (January 2026)
  - Added `<camera name="main">` to geodude.xml for consistent demo visualization
  - Side view from left, slightly elevated (azimuth=-90°, elevation=-26.5°)
- [x] Arm planning demo (`examples/arm_planning.py`) (January 2026)
  - Comprehensive demo showcasing full planning→execution pipeline
  - Dual-arm movements to random reachable poses
  - Vention base height adjustments between iterations
  - Supports kinematic/physics execution, configurable forks/iterations/seed
  - Run: `uv run mjpython examples/arm_planning.py --iterations 5`

### Next Steps

#### High Priority

1. ~~**Trajectory retiming and execution layer**~~ ✅ COMPLETED
   - ✅ Updated `config.py` with `KinematicLimits` dataclass and UR5e defaults
   - ✅ Added `trajectory.py` with `Trajectory` dataclass and TOPP-RA integration
   - ✅ Added `executor.py` with `KinematicExecutor`, `PhysicsExecutor`, placeholder `RealExecutor`
   - ✅ Updated `arm.py` to use new trajectory/executor pipeline
   - ✅ All 148 tests passing including 12 new trajectory/executor tests
   - 🎯 **Result**: Paths now automatically retimed using TOPP-RA respecting vel/acc limits

2. **Integrate mj_environment**
   - Add `robot.env` property for dynamic object management
   - Support `robot.env.add_object()`, `robot.env.remove_object()`
   - Connect to asset_manager for object metadata

3. **End-to-end pick-and-place demo**
   - Create `examples/pick_and_place.py` that demonstrates full workflow
   - Verify TSR parameters work with real objects
   - Test grasp detection and collision filtering in practice

#### Medium Priority

4. **Use cloned MjData for planning**
   - Leverage `mj_environment.Simulation.clone_data()` for collision checking
   - Create `_planning_data` clone in Arm, pass to collision checker
   - Sync clone from main data at start of each `plan()` call
   - Eliminates need for explicit state restoration after planning
   - Prevents visual artifacts during planning with active viewer

5. **Add configs directory**
   - `configs/default.yaml` - robot configuration
   - `configs/named_poses.yaml` - home, ready, etc.

6. **Add README.md**
   - Installation instructions
   - Quick start guide
   - API overview

7. **Add CI workflow**
   - GitHub Actions for pytest
   - Linting with ruff

#### Lower Priority (Future)

8. **Bimanual coordination**
   - `robot.bimanual.pick()` for two-arm manipulation
   - Synchronized planning for both arms

9. ~~**Viewer integration**~~ ✅ COMPLETED (in arm_planning demo)
   - ✅ MuJoCo passive viewer with configurable camera
   - ✅ Visual flags for cleaner appearance (disable contact force/point visualization)
   - Optional headless mode still TODO

10. **Layer 2 features**
    - Grasp verification via force sensing
    - Slip detection during transport
    - Recovery behaviors

11. **Real robot execution**
    - Implement `RealExecutor` using ur_rtde
    - Test trajectory streaming to real UR5e
    - Verify sim-to-real transfer of tuned gains

## Design Decisions

### Configuration vs TSR Input to Planner

**Problem**: When planning to a known goal configuration, should we:
1. Pass the config directly to the planner's `goal` parameter, or
2. Convert config→FK→point TSR and pass to `goal_tsrs`?

**Decision**: Always pass configurations directly via `goal` parameter.

**Rationale**:
- Converting config→pose→TSR→IK is redundant and wasteful
- IK may produce a *different* solution than the original config (UR5e has up to 8 IK solutions)
- The planner would then plan to the wrong configuration
- Direct configs are validated and used as tree roots without any IK solving

**Implementation**:
```python
# CORRECT: Direct config
path = planner.plan(start=q_start, goal=q_goal)

# WRONG: Redundant FK/IK round-trip
goal_pose = forward_kinematics(q_goal)
goal_tsr = TSR(T0_w=goal_pose, Bw=zeros)  # Point TSR
path = planner.plan(start=q_start, goal_tsrs=[goal_tsr])  # IK may give different config!
```

pycbirrt's `plan()` method handles both cases correctly:
- `goal` parameter: Configs validated and used directly as tree roots
- `goal_tsrs` parameter: TSRs sampled and IK solved to find configs

### State Restoration After Planning

**Problem**: The planner's collision checker modifies robot state during tree exploration.

**Current Solution**: Both `plan_to_configuration()` and `plan_to_tsrs()` restore robot state after planning.

```python
def plan_to_tsrs(self, tsrs, ...):
    q_start = self.get_joint_positions()
    path = planner.plan(...)

    # CRITICAL: Restore state
    self.set_joint_positions(q_start)
    for i in range(len(self.data.qvel)):
        self.data.qvel[i] = 0.0

    return path
```

**Better Solution (Future)**: Use mj_environment's cloning functionality to avoid corrupting main state entirely.

```python
# In Arm.__init__ or _get_planner():
from mj_environment import Simulation

# Create a separate MjData for planning operations
self._planning_data = Simulation.clone_data(self.data)

# Pass cloned data to collision checker
collision_checker = GraspAwareCollisionChecker(
    self.model,
    self._planning_data,  # <-- Use clone, not main data
    ...
)

# At start of each plan() call, sync clone from main state
def plan_to_tsrs(self, tsrs, ...):
    Simulation.copy_data(self._planning_data, self.data)  # Sync
    path = planner.plan(...)  # Planner mutates clone, not main data
    return path  # No restoration needed!
```

**Benefits**:
- Main robot state never corrupted during planning
- No explicit state restoration needed
- Viewer shows correct robot pose throughout planning
- Cleaner separation of concerns

### Kinematic vs Physics Execution

**Problem**: Should trajectory execution use physics simulation or direct position setting?

**Decision**: Support both via separate executor classes.

**Rationale**:
- **KinematicExecutor** (direct qpos setting):
  - Perfect tracking (0° error by definition)
  - Fast execution (no physics overhead)
  - Ideal for visualization, demos, planning validation
  - Cannot detect collisions during execution

- **PhysicsExecutor** (MuJoCo simulation):
  - Realistic tracking with actuator dynamics (~1-3° error with tuned gains)
  - Detects collisions during execution
  - Required for sim-to-real validation
  - Slower due to physics stepping

**Implementation**:
```python
# Different APIs reflect different semantics
class KinematicExecutor:
    def set_position(self, q): ...  # Immediate, no step needed

class PhysicsExecutor:
    def set_target(self, q, qd): ...  # Set control target
    def step(self): ...               # Step physics simulation
```

### Forked Parallel Planning

**Problem**: RRT is probabilistic; different seeds find different paths with varying quality/speed.

**Decision**: Support parallel planning with multiple seeds, first success wins.

**Rationale**:
- Running N planners in parallel increases probability of fast success
- Process-based parallelism avoids GIL issues with numpy/MuJoCo
- Memory overhead acceptable for N=2-4 forks
- Significant speedup on multi-core machines for difficult problems

**Implementation**:
```python
def plan_with_forks(arm, goal, num_forks=2):
    """Run multiple RRT instances, return first successful path."""
    with ProcessPoolExecutor(max_workers=num_forks) as pool:
        futures = [pool.submit(plan_single, arm, goal, seed=i) for i in range(num_forks)]
        for future in as_completed(futures):
            if (result := future.result()) is not None:
                return result  # First success wins
    return None
```

## Open Questions

1. **Bimanual planning**: Extend pycbirrt for dual-arm, or use separate planners with shared collision?
2. **Grasp ranking**: How to select best TSR when multiple are reachable?
3. ~~**Trajectory timing**: Who handles velocity/acceleration profiles?~~ → Resolved: TOPP-RA for retiming, Executor abstraction for sim/real execution
4. ~~**Viewer integration**: MuJoCo viewer, separate visualizer, or headless?~~ → Resolved: MuJoCo passive viewer with `launch_passive()`, headless still available
5. **Parallel planning strategy**: Current forked planning uses processes; would threading or async be better for lower overhead?
6. **Physics executor tracking threshold**: Current 10° max error threshold is conservative; should this be configurable per-arm or per-trajectory?

## References

- HERBPy: https://github.com/personalrobotics/herbpy (API inspiration)
- PrPy: https://github.com/personalrobotics/prpy (planning framework patterns)
- pycbirrt UR5e example: `pycbirrt/examples/ur5e_mujoco.py` (integration patterns)
- EAIK: Analytical IK for 6-DOF manipulators (up to 8 solutions)
- MuJoCo collision filtering: contype/conaffinity bitmasks
- TSR paper: Berenson, Srinivasa, Kuffner (IJRR 2011)
- TOPP-RA: https://github.com/hungpham2511/toppra (time-optimal path parameterization)
- ur_rtde: https://sdurobotics.gitlab.io/ur_rtde/ (UR robot real-time interface)
