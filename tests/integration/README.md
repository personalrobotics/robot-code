# Workspace integration tests

Cross-repo tests that exercise multiple sibling packages together. Unlike each
sibling's own `tests/`, these tests assume the full uv workspace is checked out
(i.e. `./setup.sh` has been run) and verify that the packages compose correctly.

Run them from the workspace root:

```bash
uv run pytest tests/integration -v
```

## Guidelines

- Keep tests **headless** (no display, no interactive viewer).
- Keep runtime under ~10s per test when possible.
- Prefer smoke checks (construct, step once, destroy) over full demos.
- If a test needs a real robot or GPU, skip it via `pytest.mark.skipif` so CI stays green.
