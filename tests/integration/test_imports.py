"""Verify every workspace sibling package is importable together."""

# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Siddhartha Srinivasa

import importlib

import pytest

# geodude, ada_mj and ada_assets are not here: each is its own project, apart from this environment.
SIBLINGS = [
    "asset_manager",
    "geodude_assets",
    "mj_environment",
    "mj_viser",
    "prl_assets",
    "sscbirrt",
    "tsr",
]


@pytest.mark.parametrize("name", SIBLINGS)
def test_sibling_importable(name: str) -> None:
    importlib.import_module(name)
