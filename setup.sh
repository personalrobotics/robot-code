#!/usr/bin/env bash
# Bootstrap the robot-code workspace.
# Run once after cloning this repo to clone all sibling repos and install
# geodude's and ada_mj's own environments and the shared Python environment.
set -euo pipefail

# Every sibling repo the workspace tracks. ada_mj and geodude each have their own
# environment, synced below, and ada_assets is its own project too; the ROS 2 repos (ada_ros2, ada_feeding, articutool_ros2) are not Python
# packages but are part of the workspace. This list must stay in sync with the
# tracked gitlinks — scripts/check_gitlink_freshness.sh gates that they are not
# stale. ada_feeding's default branch is ros2-devel (git clone picks it up).
REPOS=(
    "https://github.com/personalrobotics/ada_assets"
    "https://github.com/personalrobotics/ada_feeding"
    "https://github.com/personalrobotics/ada_mj"
    "https://github.com/personalrobotics/ada_ros2"
    "https://github.com/personalrobotics/articutool_ros2"
    "https://github.com/personalrobotics/asset_manager"
    "https://github.com/personalrobotics/geodude"
    "https://github.com/personalrobotics/geodude_assets"
    "https://github.com/personalrobotics/mj_environment"
    "https://github.com/personalrobotics/mj_viser"
    "https://github.com/personalrobotics/prl_assets"
    "https://github.com/personalrobotics/sscbirrt"
    "https://github.com/personalrobotics/sstsr"
)

# mujoco_menagerie is an external repo (not a Python package) needed by demos
MENAGERIE_URL="https://github.com/google-deepmind/mujoco_menagerie"

cd "$(dirname "$0")"

echo "==> Cloning repos..."
for url in "${REPOS[@]}"; do
    dir=$(basename "$url")
    # Several siblings are tracked as gitlinks (submodule-style pointers) but
    # there is no .gitmodules, so a fresh checkout leaves them as EMPTY
    # placeholder directories. A plain `[ ! -d ]` guard would see the dir and
    # skip it, leaving an empty (broken) uv workspace member. Detect a populated
    # repo by its .git entry instead, and clone into the empty placeholder.
    if [ -e "$dir/.git" ]; then
        echo "    $dir already present, skipping"
    else
        echo "    cloning $dir"
        rm -rf "$dir"        # drop the empty gitlink placeholder, if any
        git clone "$url"
    fi
done

echo ""
echo "==> Cloning mujoco_menagerie (robot models)..."
if [ ! -d "mujoco_menagerie" ]; then
    git clone "$MENAGERIE_URL"
else
    echo "    mujoco_menagerie already present, skipping"
fi

echo ""
echo "==> Removing stale per-package venvs (workspace uses shared root .venv)..."
# `uv run` inside a workspace member with its own .venv ignores the shared
# workspace venv, which is a common silent footgun. See robot-code#62.
# geodude, ada_mj and ada_assets are not members: each .venv is its own environment, so it stays.
shopt -s nullglob
for d in */.venv; do
    [ "$d" = "geodude/.venv" ] && continue
    [ "$d" = "ada_mj/.venv" ] && continue
    [ "$d" = "ada_assets/.venv" ] && continue
    echo "    removing $d"
    rm -rf "$d"
done
shopt -u nullglob

echo ""
echo "==> Installing Geodude in its own environment (geodude/, uv sync --locked)..."
# First, so it does not depend on the shared workspace resolving.
(cd geodude && uv sync --locked)

echo ""
echo "==> Installing ADA in its own environment (ada_mj/, uv sync --locked)..."
(cd ada_mj && uv sync --locked)

echo ""
echo "==> Installing Python workspace (uv sync)..."
uv sync

echo ""
echo "Done. Verify with:"
echo "  (cd geodude && uv run geodude-recycling --seed 0)"
echo "  (cd ada_mj && uv run ada-lift --seed 0)"
