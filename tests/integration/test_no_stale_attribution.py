"""Guard against regressions: no UW/PRL attribution strings in any sibling repo.

This test fails if any sibling repo reintroduces 'University of Washington',
'Personal Robotics Lab(oratory)', or the old UW email address in text files.
"""

# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Siddhartha Srinivasa

from pathlib import Path

BANNED_PATTERNS = [
    "University of Washington",
    "Personal Robotics Laboratory",
    "Personal Robotics Lab,",
    "siddh@cs.washington.edu",
]

SIBLING_DIRS = [
    "asset_manager",
    "geodude",
    "geodude_assets",
    "mj_environment",
    "mj_manipulator",
    "mj_manipulator_ros",
    "mj_viser",
    "prl_assets",
    "pycbirrt",
    "tsr",
]

TEXT_SUFFIXES = {".py", ".md", ".toml", ".yaml", ".yml", ".txt", ".cfg", ".ini", ".xml"}
SKIP_DIR_NAMES = {".git", ".venv", "__pycache__", "node_modules", "mujoco_menagerie", "build", "dist"}


def test_no_banned_attribution() -> None:
    root = Path(__file__).resolve().parents[2]
    hits: list[str] = []
    for sibling in SIBLING_DIRS:
        sib_dir = root / sibling
        if not sib_dir.is_dir():
            continue
        for path in sib_dir.rglob("*"):
            if not path.is_file() or path.suffix not in TEXT_SUFFIXES:
                continue
            if any(part in SKIP_DIR_NAMES for part in path.parts):
                continue
            try:
                text = path.read_text(encoding="utf-8", errors="ignore")
            except OSError:
                continue
            for pat in BANNED_PATTERNS:
                if pat in text:
                    hits.append(f"{path.relative_to(root)}: {pat!r}")
    assert not hits, "Banned attribution strings found:\n  " + "\n  ".join(hits)
