# MuJoCo group conventions

Every MuJoCo model in this workspace (`geodude_assets`, `ada_assets`, `prl_assets`, and anything added later) assigns geom and site groups the same way. Groups decide what a viewer shows by default. MuJoCo's own viewer, and mj_viser, show groups 0–2 and hide 3–5. So the group of an element encodes whether people normally see it.

The convention follows MuJoCo Menagerie, so robots imported from Menagerie usually comply already.

## Geoms

| Group | Holds | Shown by default |
| --- | --- | --- |
| 0 | Geometry that both renders and collides: floors, tables, simple props with no separate visual mesh | yes |
| 2 | Visual geometry: meshes meant to be seen, with `contype="0" conaffinity="0"` | yes |
| 3 | Collision geometry: proxies used by physics, and physics-disabled volumes used by software checks (safety envelopes, planning volumes) | no |
| 1, 4, 5 | Not used | |

- **Separate visual and collision geometry:** if a body has a visual mesh, its collision geometry goes in **3**, never 0. Otherwise collision proxies draw on top of the visual mesh.
- **Physics-disabled volumes** (`contype="0" conaffinity="0"`) are visuals (2) unless they exist for collision checking, in which case they go in 3. An example is ADA's `body_safety_envelope`.

## Sites

| Group | Holds | Shown by default | Examples |
| --- | --- | --- | --- |
| 0 | Task frames people work with | yes | grasp sites, end-effector and tool-tip frames, feeding targets (`ee_site`, `grasp_site`, `fork_tip`, `mouth`) |
| 4 | Engineering frames | no | attachment and mount points, F/T and IMU sensor frames, camera links and optical frames, force-sensor pads, invisible region markers |
| 1, 2, 3, 5 | Not used | | |

## Viewing

Defaults follow the convention: people see visuals, render-and-collide geometry, and task frames. Collision geometry and engineering frames are one toggle away in the viewer's Visibility panel. An application that wants a debug preset enables specific groups, for example `viewer.set_visible_groups(sites={0, 1, 2, 4})` in mj_viser, rather than every group.

## Checking

`scripts/check_mujoco_groups.py MODEL.xml ...` reports every geom and site whose group breaks these rules. `tests/integration/test_mujoco_groups.py` runs it on every model in the asset packages. Models that do not comply yet are marked as expected failures, each linked to the issue that tracks the fix.

The checker cannot tell a task frame from an engineering frame; it only enforces that sites use group 0 or 4. Classifying sites is a review responsibility.
