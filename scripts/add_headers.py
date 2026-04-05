#!/usr/bin/env python3
"""Prepend SPDX + copyright header to every .py file in one or more directories.

Usage:
    scripts/add_headers.py --dry-run <repo_dir> [<repo_dir> ...]
    scripts/add_headers.py --apply   <repo_dir> [<repo_dir> ...]

The header is two lines:
    # SPDX-License-Identifier: MIT
    # Copyright (c) 2025 Siddhartha Srinivasa

Files already containing "SPDX-License-Identifier" are skipped. Shebangs and
coding declarations on lines 1-2 are preserved.
"""

# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Siddhartha Srinivasa

from __future__ import annotations

import argparse
import sys
from pathlib import Path

HEADER_LINES = [
    "# SPDX-License-Identifier: MIT",
    "# Copyright (c) 2025 Siddhartha Srinivasa",
]
SKIP_DIR_NAMES = {
    ".git",
    ".venv",
    "__pycache__",
    "node_modules",
    "build",
    "dist",
    ".eggs",
}


def should_skip(path: Path) -> bool:
    return any(part in SKIP_DIR_NAMES for part in path.parts)


def add_header(path: Path, *, apply: bool) -> str:
    """Return one of: 'added', 'present', 'empty'."""
    text = path.read_text(encoding="utf-8")
    if "SPDX-License-Identifier" in text:
        return "present"
    lines = text.splitlines(keepends=True)
    if not lines:
        return "empty"

    # Preserve shebang and optional coding declaration.
    prefix: list[str] = []
    i = 0
    if lines and lines[0].startswith("#!"):
        prefix.append(lines[0])
        i = 1
    if (
        i < len(lines)
        and ("coding:" in lines[i] or "coding=" in lines[i])
        and lines[i].lstrip().startswith("#")
    ):
        prefix.append(lines[i])
        i += 1

    new_header = "\n".join(HEADER_LINES) + "\n"
    rest = "".join(lines[i:])
    # Ensure a blank line between header and rest if rest is non-empty and does not already start with one.
    if rest and not rest.startswith("\n"):
        new_header += "\n"
    new_text = "".join(prefix) + new_header + rest

    if apply:
        path.write_text(new_text, encoding="utf-8")
    return "added"


def main() -> int:
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--dry-run", action="store_true")
    mode.add_argument("--apply", action="store_true")
    parser.add_argument("roots", nargs="+", type=Path)
    args = parser.parse_args()

    added = 0
    present = 0
    empty = 0
    for root in args.roots:
        if not root.exists():
            print(f"[warn] missing: {root}", file=sys.stderr)
            continue
        for py in root.rglob("*.py"):
            if should_skip(py):
                continue
            result = add_header(py, apply=args.apply)
            if result == "added":
                added += 1
                prefix = "[add] " if args.apply else "[would-add] "
                print(f"{prefix}{py}")
            elif result == "present":
                present += 1
            else:
                empty += 1

    verb = "added" if args.apply else "would add"
    print(f"\nSummary: {verb}={added}  already-present={present}  empty={empty}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
