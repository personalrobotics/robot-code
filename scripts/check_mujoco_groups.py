#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Siddhartha Srinivasa

"""Check MuJoCo models against the workspace group convention (docs/mujoco-groups.md).

Usage:
    scripts/check_mujoco_groups.py MODEL.xml [MODEL.xml ...]

Prints one line per violation and exits 1 if any model has one.
"""

from __future__ import annotations

import sys

import mujoco

VISUAL_GROUP = 2
COLLISION_GROUP = 3
RENDER_AND_COLLIDE_GROUP = 0
SITE_GROUPS = {0, 4}  # task frames, engineering frames


def _name(model: mujoco.MjModel, obj: mujoco.mjtObj, index: int, prefix: str) -> str:
    return mujoco.mj_id2name(model, obj, index) or f"{prefix}{index}"


def violations(model: mujoco.MjModel) -> list[str]:
    """Every geom and site whose group breaks the convention, with the rule it breaks."""
    found = []
    visual_only = [model.geom_contype[g] == 0 and model.geom_conaffinity[g] == 0 for g in range(model.ngeom)]
    # Bodies that carry a visual (any non-colliding geom that is not a group-3 collision volume),
    # even a misplaced one, so every violation is reported in one pass.
    bodies_with_visuals = {
        int(model.geom_bodyid[g])
        for g in range(model.ngeom)
        if visual_only[g] and model.geom_group[g] != COLLISION_GROUP
    }
    for g in range(model.ngeom):
        group = int(model.geom_group[g])
        name = _name(model, mujoco.mjtObj.mjOBJ_GEOM, g, "geom")
        body = _name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[g]), "body")
        if visual_only[g]:
            # Physics-disabled geoms are visuals (2) or software-checked collision volumes (3).
            if group not in (VISUAL_GROUP, COLLISION_GROUP):
                found.append(
                    f"geom {name} (body {body}): non-colliding geom in group {group}; "
                    f"use {VISUAL_GROUP} (visual) or {COLLISION_GROUP} (collision volume)"
                )
        elif group == RENDER_AND_COLLIDE_GROUP:
            if int(model.geom_bodyid[g]) in bodies_with_visuals:
                found.append(
                    f"geom {name} (body {body}): collision geom in group 0 beside a visual mesh; "
                    f"use {COLLISION_GROUP}"
                )
        elif group != COLLISION_GROUP:
            found.append(f"geom {name} (body {body}): colliding geom in group {group}; use {COLLISION_GROUP} or 0")
    for s in range(model.nsite):
        group = int(model.site_group[s])
        if group not in SITE_GROUPS:
            name = _name(model, mujoco.mjtObj.mjOBJ_SITE, s, "site")
            found.append(f"site {name}: group {group}; use 0 (task frame) or 4 (engineering frame)")
    return found


def main(paths: list[str]) -> int:
    failed = False
    for path in paths:
        for line in violations(mujoco.MjModel.from_xml_path(path)):
            print(f"{path}: {line}")
            failed = True
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
