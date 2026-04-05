#!/usr/bin/env bash
# Sync canonical templates from .github-templates/ into each sibling repo.
#
# Usage:
#   scripts/sync_templates.sh --dry-run              # show what would change (all repos)
#   scripts/sync_templates.sh --apply                # apply to all repos
#   scripts/sync_templates.sh --apply --repo=tsr     # apply to one repo
#
# The script copies files from .github-templates/ to each sibling. It does NOT
# touch each sibling's pyproject.toml, README, LICENSE text, or source headers
# — those are per-repo and handled separately.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
TEMPLATES="$ROOT/.github-templates"

SIBLINGS=(
    asset_manager
    geodude
    geodude_assets
    mj_environment
    mj_manipulator
    mj_manipulator_ros
    mj_viser
    prl_assets
    pycbirrt
    tsr
)

# Files to copy from .github-templates/ into each sibling (src -> dst).
# Left column is the path under .github-templates/; right column is the
# destination path under the sibling repo.
FILES=(
    "CODEOWNERS:.github/CODEOWNERS"
    "pull_request_template.md:.github/pull_request_template.md"
    "ISSUE_TEMPLATE/bug_report.md:.github/ISSUE_TEMPLATE/bug_report.md"
    "ISSUE_TEMPLATE/improvement.md:.github/ISSUE_TEMPLATE/improvement.md"
    "ISSUE_TEMPLATE/task.md:.github/ISSUE_TEMPLATE/task.md"
    "ISSUE_TEMPLATE/config.yml:.github/ISSUE_TEMPLATE/config.yml"
    "workflows/ci.yml:.github/workflows/ci.yml"
    "CONTRIBUTING.md:CONTRIBUTING.md"
    "SECURITY.md:SECURITY.md"
    "CODE_OF_CONDUCT.md:CODE_OF_CONDUCT.md"
    "LICENSE:LICENSE"
)

MODE="dry-run"
ONLY_REPO=""
for arg in "$@"; do
    case "$arg" in
        --dry-run) MODE="dry-run" ;;
        --apply) MODE="apply" ;;
        --repo=*) ONLY_REPO="${arg#--repo=}" ;;
        -h|--help)
            sed -n '2,10p' "$0" | sed 's/^# \{0,1\}//'
            exit 0
            ;;
        *) echo "unknown arg: $arg" >&2; exit 2 ;;
    esac
done

sync_one() {
    local sibling="$1"
    local sib_dir="$ROOT/$sibling"
    if [ ! -d "$sib_dir" ]; then
        echo "  [skip] $sibling (not cloned)"
        return
    fi
    echo "== $sibling =="
    for pair in "${FILES[@]}"; do
        local src_rel="${pair%%:*}"
        local dst_rel="${pair##*:}"
        local src="$TEMPLATES/$src_rel"
        local dst="$sib_dir/$dst_rel"
        if [ ! -f "$src" ]; then
            echo "  [warn] missing template: $src_rel"
            continue
        fi
        if [ -f "$dst" ] && cmp -s "$src" "$dst"; then
            # Already in sync.
            continue
        fi
        if [ "$MODE" = "apply" ]; then
            mkdir -p "$(dirname "$dst")"
            cp "$src" "$dst"
            echo "  [sync] $dst_rel"
        else
            if [ -f "$dst" ]; then
                echo "  [diff] $dst_rel"
            else
                echo "  [new]  $dst_rel"
            fi
        fi
    done
}

if [ -n "$ONLY_REPO" ]; then
    sync_one "$ONLY_REPO"
else
    for s in "${SIBLINGS[@]}"; do
        sync_one "$s"
    done
fi

if [ "$MODE" = "dry-run" ]; then
    echo ""
    echo "Dry run only. Re-run with --apply to make changes."
fi
