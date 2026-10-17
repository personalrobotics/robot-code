"""Every asset model follows the workspace group convention (docs/mujoco-groups.md)."""

# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Siddhartha Srinivasa

import importlib.util
from pathlib import Path

import mujoco
import pytest

_CHECKER = Path(__file__).resolve().parents[2] / "scripts" / "check_mujoco_groups.py"
_spec = importlib.util.spec_from_file_location("check_mujoco_groups", _CHECKER)
assert _spec is not None and _spec.loader is not None
check_mujoco_groups = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(check_mujoco_groups)


def _models() -> list[tuple[str, Path]]:
    import ada_assets
    import geodude_assets
    import prl_assets

    models = [
        ("geodude_assets", geodude_assets.MODELS_DIR / rel)
        for rel in (
            "geodude/geodude.xml",
            "universal_robots_ur5e/ur5e.xml",
            "robotiq_2f140/2f140.xml",
            "abh_left_small/abh_left_small.xml",
            "abh_right_small/abh_right_small.xml",
            "vention/vention.xml",
        )
    ]
    # ADA's component files rely on the assembled models to set their mesh directory.
    models += [("ada_assets", ada_assets.MODELS_DIR / rel) for rel in ("ada.xml", "scene.xml")]
    models += [("prl_assets", path) for path in sorted(prl_assets.OBJECTS_DIR.glob("*/*.xml"))]
    return models


# Models that do not comply yet, with the issue tracking each fix. These are strict
# expected failures: once a model is fixed, its test passes and the entry must go.
KNOWN_VIOLATIONS: dict[str, str] = {}


def _id(package: str, path: Path) -> str:
    return f"{package}/{path.parent.name}/{path.name}"


def _params() -> list[object]:
    params = []
    for package, path in _models():
        model_id = _id(package, path)
        marks = []
        if model_id in KNOWN_VIOLATIONS:
            marks.append(pytest.mark.xfail(strict=True, reason=KNOWN_VIOLATIONS[model_id]))
        params.append(pytest.param(path, id=model_id, marks=marks))
    return params


@pytest.mark.parametrize("path", _params())
def test_groups_follow_convention(path: Path) -> None:
    violations = check_mujoco_groups.violations(mujoco.MjModel.from_xml_path(str(path)))
    assert not violations, "\n".join(violations)
