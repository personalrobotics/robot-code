# Contributing

This is the **robot-code workspace** — the umbrella that ties together all
the [personalrobotics](https://github.com/personalrobotics) packages into a
single [uv workspace](https://docs.astral.sh/uv/concepts/workspaces/).

## Development setup

```bash
git clone https://github.com/personalrobotics/robot-code
cd robot-code
./setup.sh
```

`setup.sh` clones every sibling repo into the workspace and runs `uv sync`.
You can then work in any sibling's directory.

## Running tests and linters

Per-repo (from inside a sibling):

```bash
cd <repo>
uv run pytest tests/ -v
uv run ruff check .
uv run ruff format --check .
```

Cross-repo integration (from the workspace root):

```bash
uv run pytest tests/integration -v
```

## Pull requests

- Branch from `main`. Open a PR with a clear summary and test plan.
- Per-repo CI (ruff + pytest) must pass.
- Cross-repo integration CI runs automatically when any sibling's `main`
  advances, and on a nightly schedule.
- Review is by [@siddhss5](https://github.com/siddhss5) (enforced via CODEOWNERS).

## Package manager: uv only

We use [uv](https://docs.astral.sh/uv/) exclusively. **Do not use pip.**
The workspace layout in robot-code relies on uv's workspace resolution.

## License

By contributing, you agree that your contributions will be licensed under the
MIT License (see [LICENSE](LICENSE)).
